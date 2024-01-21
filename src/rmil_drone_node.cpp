#include <memory>
#include <unistd.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <arp/Autopilot.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/StatePublisher.hpp>
#include <arp/VisualInertialTracker.hpp>

#include <arp/InteractiveMarkerServer.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>

#include <arp/kinematics/operators.hpp>

#define FONT cv::FONT_HERSHEY_COMPLEX_SMALL
#define FONT_COLOR cv::Scalar(0, 255, 0)
#define FONT_OFFSET 20
#define FONT_OFFSET_PER_LETTER 11
#define CV_AA cv::LINE_AA // maintains backward compatibility with older OpenCV

std_msgs::Float64MultiArray message;

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(!vit_)
      return;
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll + msg->header.stamp.nsec / 1000;
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    vit_->addImage(timeMicroseconds, lastImage_);
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if(lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void setVisualInertialTracker(arp::VisualInertialTracker* vit){
    vit_ = vit;
  }

  void setInteractiveMarkerServer(arp::InteractiveMarkerServer* marker){
    marker_ = marker;
  }

  void setAutopilot(arp::Autopilot* autopilot){
    autopilot_ = autopilot;
  }

  void setResetPublisher(ros::Publisher* publisher){
    publisher_ = publisher;
  }

  void setHapticsPose(){
    xPose = vit_->getEstimator()->x_.t_WS[0];
    yPose = vit_->getEstimator()->x_.t_WS[1];
    zPose = vit_->getEstimator()->x_.t_WS[2];
    yawPose = arp::kinematics::yawAngle(vit_->getEstimator()->x_.q_WS);
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    if(!vit_)
      return;
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll + msg->header.stamp.nsec / 1000;
    Eigen::Vector3d acc_S, omega_S;
    acc_S << msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z;
    omega_S << msg->angular_velocity.x,
               msg->angular_velocity.y,
               msg->angular_velocity.z;
    vit_->addImuMeasurement(timeMicroseconds, omega_S, acc_S);
  }

  void hapticsCallback(const std_msgs::Float64MultiArray& msg)
  {
    if(!marker_)
        return;

    
    //Matrix for transforming from world frame to robot frame
    const Eigen::Matrix3d R_WS = vit_->getEstimator()->x_.q_WS.toRotationMatrix();

    ///Control message to stop updating pose and set new center position of joystick
    if(msg.data[5]){
      autopilot_->setPoseReference(poseRobot[0], poseRobot[1], poseRobot[2], yawPose);
      marker_->activate(poseRobot[0], poseRobot[1], poseRobot[2], yawPose);
      
      autopilot_-> delta_x = 0;
      autopilot_-> delta_y = 0;
      autopilot_-> delta_z = 0;
      std_msgs::Int16 resMsg;
      resMsg.data = 1; 
      publisher_->publish(resMsg);
      return;
    }
    ///If control message is 0, use other values to update pose
    //Update reference values for pose
    xPose -= msg.data[0]*0.08; 
    yPose -= msg.data[1]*0.08;
    zPose += msg.data[2]*0.08;

    if(msg.data[3]){
      yawPose+=0.05;
    }
    if(msg.data[4]){
      yawPose-=0.05;
    }
    //save haptic device deltas for force feedback
    autopilot_-> delta_x = msg.data[0];
    autopilot_-> delta_y = msg.data[1];
    autopilot_-> delta_z = msg.data[2];
    //set new pose reference and update interactive marker in robot frame
    poseRobot = {xPose, yPose, zPose};
    poseRobot = R_WS * poseRobot;
    autopilot_->setPoseReference(poseRobot[0], poseRobot[1], poseRobot[2], yawPose);
    marker_->activate(poseRobot[0], poseRobot[1], poseRobot[2], yawPose);
    return;
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
  arp::VisualInertialTracker* vit_ = nullptr;
  arp::InteractiveMarkerServer* marker_ = nullptr;
  arp::Autopilot* autopilot_ = nullptr;
  ros::Publisher* publisher_ = nullptr;

  double xPose;
  double yPose;
  double zPose;
  double yawPose;
  Eigen::Vector3d poseRobot;
};

/// \brief Read occupancy grid from file.
/// \param filename File to read.
bool readOccupancyMap(const std::string& filename, cv::Mat& occupancyMap)
{
  // open the file:
  std::ifstream mapFile(filename, std::ios::in | std::ios::binary);
  if(!mapFile.is_open()) {
    ROS_FATAL_STREAM("could not open map file " << filename);
    return false;
  }
  // first read the map size along all the dimensions:
  int mapSizes[3];
  if(!mapFile.read((char *)mapSizes, 3* sizeof(int))) {
    ROS_FATAL_STREAM("could not read map file " << filename);
    return false;
  }
  
  // create the returned cv::Mat:
  occupancyMap = cv::Mat(3, mapSizes, CV_8SC1);

  // now read the map data: don’t forget to delete[] in the end!
  int numCells = mapSizes[0] * mapSizes[1] * mapSizes[2];
  if(!mapFile.read((char *)occupancyMap.data, numCells)) {
    ROS_FATAL_STREAM("could not read map file " << filename);
    return false;
  }
  mapFile.close();
  return true;
}

/// \brief Annotate the image with the state of the autopilot, for debugging.
/// @param[in] image Image to annotate, will be overwritten.
/// @param[in] autopilot Autopilot for retrieving state information.
void annotateImage(cv::Mat& image, arp::Autopilot& autopilot)
{
  const auto status = autopilot.getDroneStatus();
  if(status == arp::Autopilot::DroneStatus::Unknown) {
    ROS_DEBUG("Drone status unknown");
    return;
  }
  cv::Mat legend(image.rows / 5, image.cols, image.type(), cv::Scalar(255, 255, 255));
  int x = 10;
  int y = 10;
  const int step = 1.5 * 10;

  //Legend display
  cv::putText(legend, "Use haptic device as joystick to command drone in 3 coordinate directions", cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, 1, 1, CV_AA);
  cv::putText(legend, "Use left and right button for yawing", cv::Point(x, y + step), cv::FONT_HERSHEY_PLAIN, 1, 1, 1, CV_AA);
  cv::putText(legend, "Use center button to stop commanding and setting new joystick center", cv::Point(x, y + 2 * step), cv::FONT_HERSHEY_PLAIN, 1, 1, 1, CV_AA);
  cv::putText(legend, "Battery state:" + std::to_string(autopilot.getBatteryStatus()), cv::Point(x, y + 3 * step), cv::FONT_HERSHEY_PLAIN, 1, 1, 1, CV_AA);
  
  cv::vconcat(image, legend, image);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rmil_drone_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  double fu, fv, cu, cv, k1, k2, p1, p2, focalLengthMap;
  if(!ros::param::get("~fu", fu)) ROS_FATAL("error loading parameter fu");
  if(!ros::param::get("~fv", fv)) ROS_FATAL("error loading parameter fv");
  if(!ros::param::get("~cu", cu)) ROS_FATAL("error loading parameter cu");
  if(!ros::param::get("~cv", cv)) ROS_FATAL("error loading parameter cv");
  if(!ros::param::get("~k1", k1)) ROS_FATAL("error loading parameter k1");
  if(!ros::param::get("~k2", k2)) ROS_FATAL("error loading parameter k2");
  if(!ros::param::get("~p1", p1)) ROS_FATAL("error loading parameter p1");
  if(!ros::param::get("~p2", p2)) ROS_FATAL("error loading parameter p2");
  if(!ros::param::get("~focalLengthMap", focalLengthMap)) ROS_FATAL("error loading parameter focalLengthMap");

  std::string path = ros::package::getPath("rmil_drone_practicals");
  std::string mapFile;
  if(!ros::param::get("~map", mapFile))
    ROS_FATAL("error loading parameter for map");
  std::string mapPath = path + "/maps/" + mapFile;

  Eigen::Matrix4d T_SC_mat;
  std::vector<double> T_SC_array;
  if(!ros::param::get("~T_SC", T_SC_array))
    ROS_FATAL("error loading parameter T_SC");
  T_SC_mat <<
    T_SC_array[0], T_SC_array[1], T_SC_array[2], T_SC_array[3],
    T_SC_array[4], T_SC_array[5], T_SC_array[6], T_SC_array[7],
    T_SC_array[8], T_SC_array[9], T_SC_array[10], T_SC_array[11],
    T_SC_array[12], T_SC_array[13], T_SC_array[14], T_SC_array[15];
  arp::kinematics::Transformation T_SC(T_SC_mat);

  double briskUniformityRadius, briskAbsoluteThreshold;
  int briskMatchingThreshold, briskOctaves, briskMaxNumKeypoints;
  float reprojectionDistThreshold;
  if(!ros::param::get("~briskMatchingThreshold", briskMatchingThreshold)) ROS_FATAL("error loading parameter briskMatchingThreshold");
  if(!ros::param::get("~briskUniformityRadius", briskUniformityRadius)) ROS_FATAL("error loading parameter briskUniformityRadius");
  if(!ros::param::get("~briskOctaves", briskOctaves)) ROS_FATAL("error loading parameter briskOctaves");
  if(!ros::param::get("~briskAbsoluteThreshold", briskAbsoluteThreshold)) ROS_FATAL("error loading parameter briskAbsoluteThreshold");
  if(!ros::param::get("~briskMaxNumKeypoints", briskMaxNumKeypoints)) ROS_FATAL("error loading parameter briskMaxNumKeypoints");
  if(!ros::param::get("~reprojectionDistanceThreshold", reprojectionDistThreshold)) ROS_FATAL("error loading parameter reprojectionDistanceThreshold");

  // Global settings.
  bool isImageUndistorted = false;

  // set up autopilot and visual-inertial tracking.
  arp::Autopilot autopilot(nh);

  double x, y, z, yaw;
  autopilot.getPoseReference(x, y, z, yaw);
  arp::InteractiveMarkerServer markerServer(autopilot);
  markerServer.activate(x, y, z, yaw);
  
  cv::Mat occupancyMap;
  // Link for Drone Lab:
  //readOccupancyMap("/home/student/rmil_drone_ws/src/rmil_drone_practicals/maps/occupancy-map_skokloster.dat", occupancyMap);
  //Link for personal laptop:
  readOccupancyMap("/home/pablo/rmil_drone_ws/src/rmil_drone_practicals/maps/occupancy-map_skokloster.dat", occupancyMap);
  autopilot.occupancy_map = occupancyMap;
  

  arp::Frontend frontend(640, 360, fu, fv, cu, cv, k1, k2, p1, p2, focalLengthMap,
                         briskMatchingThreshold, briskUniformityRadius, briskOctaves,
                         briskAbsoluteThreshold, briskMaxNumKeypoints, reprojectionDistThreshold);
  if(!frontend.loadMap(mapPath))
    ROS_FATAL_STREAM("could not load map from " << mapPath << " !");
  std::string vocPath = path + "/maps/small_voc.yml.gz";
  if(!frontend.loadDBoW2Voc(vocPath))
    ROS_FATAL_STREAM("could not load DBoW vocabulary from " << vocPath << " !");
  if(!frontend.fillDBoW2Database())
    ROS_FATAL_STREAM("could not fill DBoW2 with map landmarks!");

  arp::ViEkf viEkf;
  viEkf.setCameraExtrinsics(T_SC);
  viEkf.setCameraIntrinsics(frontend.camera());

  arp::VisualInertialTracker visualInertialTracker;
  visualInertialTracker.setFrontend(frontend);
  visualInertialTracker.setEstimator(viEkf);
  
  visualInertialTracker.setControllerCallback(std::bind(&arp::Autopilot::controllerCallback, &autopilot, std::placeholders::_1, std::placeholders::_2));

  // setup inputs and outputs.
  Subscriber subscriber;
  subscriber.setAutopilot(&autopilot);
  subscriber.setVisualInertialTracker(&visualInertialTracker);
  subscriber.setInteractiveMarkerServer(&markerServer);
  
  //Ros Publisher to set new "center" position of joystick
  ros::Publisher pubPoseReset = nh.advertise<std_msgs::Int16>("/dronePose/reset", 1, false);
  subscriber.setResetPublisher(&pubPoseReset);
  
  image_transport::Subscriber subImage = it.subscribe("ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50, &Subscriber::imuCallback, &subscriber);
  ros::Subscriber subHaptics = nh.subscribe("haptics/pos_cmd", 1, &Subscriber::hapticsCallback, &subscriber);
  arp::StatePublisher pubState(nh);
  visualInertialTracker.setVisualisationCallback(std::bind( &arp::StatePublisher::publish, &pubState, std::placeholders::_1, std::placeholders::_2));

  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, 640, 480, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // enter main event loop
  ROS_INFO_STREAM("===== Hello AR Drone ====");
  cv::Mat image;
  std::vector <Eigen::Vector3d> trajectory;

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if(event.type == SDL_QUIT) {
      break;
    }

    // render image, if there is a new one available
    if(visualInertialTracker.getLastVisualisationImage(image))
    {
      if(isImageUndistorted && !frontend.camera().undistortImage(image, image)) {
        ROS_WARN("Image undistortion failed");
      }
      annotateImage(image, autopilot);

      // Resize the renderer to the image size. The rendered image will be
      // scaled to fit the window.
      SDL_RenderSetLogicalSize(renderer, image.cols, image.rows);
      // https://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
      // I'm using SDL_TEXTUREACCESS_STREAMING because it's for a video player, you should
      // pick whatever suits you most: https://wiki.libsdl.org/SDL_TextureAccess
      // remember to pick the right SDL_PIXELFORMAT_* !
      texture = SDL_CreateTexture(
          renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, image.cols, image.rows);
      SDL_UpdateTexture(texture, nullptr, (void*)image.data, image.step1());
      SDL_RenderClear(renderer);
      SDL_RenderCopy(renderer, texture, nullptr, nullptr);
      SDL_RenderPresent(renderer);
      // cleanup (only after you're done displaying. you can repeatedly call UpdateTexture without destroying it)
      SDL_DestroyTexture(texture);
    }

    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(nullptr);

    // check states!
    auto droneStatus = autopilot.getDroneStatus();
    // command
    if(state[SDL_SCANCODE_ESCAPE])
    {
      bool success = autopilot.estopReset();
      ROS_INFO_STREAM("ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus << (success ? " [ OK ]" : " [FAIL]"));
    }
    if(state[SDL_SCANCODE_T])
    {
      bool success = autopilot.takeoff();
      ROS_INFO_STREAM("Taking off...                          status=" << droneStatus << (success ? " [ OK ]" : " [FAIL]"));
    }

    if(state[SDL_SCANCODE_L])
    {
      bool success = autopilot.land();
      ROS_INFO_STREAM("Going to land...                       status=" << droneStatus << (success ? " [ OK ]" : " [FAIL]"));
    }
    if(state[SDL_SCANCODE_C])
    {
      bool success = autopilot.flattrimCalibrate();
      ROS_INFO_STREAM("Requesting flattrim calibration...     status=" << droneStatus << (success ? " [ OK ]" : " [FAIL]"));
    }
    if(state[SDL_SCANCODE_U])
    {
      ROS_INFO_STREAM("Toggling image undistortion...");
      isImageUndistorted = !isImageUndistorted;
    }
    if(state[SDL_SCANCODE_RCTRL])
    {
      ROS_INFO_STREAM("Enabling automatic control...");
      subscriber.setHapticsPose();
      autopilot.setAutomatic();
      double x, y, z, yaw;
      autopilot.getPoseReference(x, y, z, yaw);
      markerServer.activate(x, y, z, yaw);
    }
    if(state[SDL_SCANCODE_SPACE])
    {
      ROS_INFO_STREAM("Enabling manual control...");
      autopilot.setManual();
    }
    if(!autopilot.isAutomatic() && autopilot.isFlying())
    {
      // TODO: read move commands and execute them!
      double up=0.0;
      double forward=0.0;
      double left =0.0;
      double rotateLeft =0.0;

    /*
    if(message.data[0]>0)
      {
        std::cout << "Moving forwards...                       status=" << droneStatus;
        forward = 1.0;
      }
    if(message.data[0]<0)
      {
        std::cout << "Moving backwards...                       status=" << droneStatus;
        forward = -1.0;
      }
    
    if(message.data[1]>0)
      {
        std::cout << "Moving left...                       status=" << droneStatus;
        left = 1.0;
      }
    if(message.data[1]<0)
      {
        std::cout << "Moving right...                       status=" << droneStatus;
        left = 1.0;
      }

    if(message.data[2]>0)
      {
        std::cout << "Moving up...                       status=" << droneStatus;
        up = 1.0;
      }
    if(message.data[2]<0)
      {
        std::cout << "Moving down...                       status=" << droneStatus;
        up = 1.0;
      }

    if(message.data[3]>0)
    {
        std::cout << "Yawing left...                       status=" << droneStatus;
        rotateLeft =  1.0;
    }
    if(message.data[4]>0)
    {
        std::cout << "Yawing right...                       status=" << droneStatus;
        rotateLeft = -1.0;
    }
    if(message.data[5]>0)
    {
      forward=0;
      left=0;
      up=0;
      rotateLeft=0;

      std_msgs::Int16 resMsg;
      resMsg.data = 1; 
      pubPoseReset.publish(resMsg);
    }
    bool success = autopilot.manualMove(forward, left, up, rotateLeft);*/
      if(state[SDL_SCANCODE_W])
      {
        ROS_INFO_STREAM("Moving up...                       status=" << droneStatus);
        up = 1.0;
      }
      if(state[SDL_SCANCODE_A])
      {
        ROS_INFO_STREAM("Yawing left...                       status=" << droneStatus);
        rotateLeft = 1.0;
      }
      if(state[SDL_SCANCODE_S])
      {
        ROS_INFO_STREAM("Moving down...                       status=" << droneStatus);
        up = -1.0;
      }
      if(state[SDL_SCANCODE_D])
      {
        ROS_INFO_STREAM("Yawing right...                       status=" << droneStatus);
        rotateLeft = -1.0;
      }
      if(state[SDL_SCANCODE_UP])
      {
        ROS_INFO_STREAM("Moving forward...                       status=" << droneStatus);
        forward = 1.0;
      }
      if(state[SDL_SCANCODE_LEFT])
      {
        ROS_INFO_STREAM("Moving left...                       status=" << droneStatus);
        left = 1.0;
      }
      if(state[SDL_SCANCODE_DOWN])
      {
        ROS_INFO_STREAM("Moving backward...                       status=" << droneStatus);
        forward = -1.0;
      }
      if(state[SDL_SCANCODE_RIGHT])
      {
        ROS_INFO_STREAM("Moving right...                       status=" << droneStatus);
        left = -1.0;
      }      
      bool success = autopilot.manualMove(forward, left, up, rotateLeft);
    }
  } 

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

