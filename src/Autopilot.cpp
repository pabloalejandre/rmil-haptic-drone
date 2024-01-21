/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <geometry_msgs/PoseStamped.h>

#include <arp/Autopilot.hpp>
#include <arp/kinematics/operators.hpp>
#include <arp/PidController.hpp>
#include <arp/kinematics/Transformation.hpp>

#include <math.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Float64MultiArray.h>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  // Receiving navdata.
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback, this);
  // Publishers.
  isAutomatic_ = false; // always start in manual mode

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  pubForce_ = nh.advertise<std_msgs::Float64MultiArray>("/drone/feedback_force", 1, false);
  
  
  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(nh_->resolveName("ardrone/flattrim"), 1);

  PidController::Parameters x_param;
  x_param.k_p =0.2;
  x_param.k_i =0.0;
  x_param.k_d =0.05;

  PidController::Parameters y_param;
  y_param.k_p =0.2;
  y_param.k_i =0.0;
  y_param.k_d =0.05;
  
  PidController::Parameters z_param;
  z_param.k_p =1.0;
  z_param.k_i =0.0;
  z_param.k_d =0.0;

  PidController::Parameters yaw_param;
  yaw_param.k_p =1.5;
  yaw_param.k_i =0.0;
  yaw_param.k_d =0.0;


  xController.setParameters(x_param);
  yController.setParameters(y_param);
  zController.setParameters(z_param);
  yawController.setParameters(yaw_param);
}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = getDroneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = getDroneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = getDroneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

Autopilot::DroneStatus Autopilot::getDroneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

float Autopilot::getBatteryStatus() const
{
  return lastNavdata_.batteryPercent;
}

bool Autopilot::isFlying() const
{
  const uint status = lastNavdata_.state;
  return status == 3 || status == 4 || status == 7;
}

bool Autopilot::isLanded() const
{
  const uint status = lastNavdata_.state;
  return status == 2;
}

bool Autopilot::manualMove(double forward, double left, double up, double rotateLeft)
{
  if(isAutomatic()) return false;
  return move(forward, left, up, rotateLeft);
}

// Move the drone.
bool Autopilot::move(double forward, double left, double up, double rotateLeft)
{
  //TODO Implement movement of the drone
  DroneStatus status = getDroneStatus();

  // ARdrone -> move
  geometry_msgs::Twist moveMsg;
  moveMsg.linear.x = forward;
  moveMsg.linear.y = left;
  moveMsg.linear.z = up;
  moveMsg.angular.z = rotateLeft;
  pubMove_.publish(moveMsg);
  return true;
}

// Set to automatic control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  ref_x_ = x;
  ref_y_ = y;
  ref_z_ = z;
  ref_yaw_ = yaw;
  return true;
}

bool Autopilot::getPoseReference(double& x, double& y, double& z, double& yaw) {
  std::lock_guard<std::mutex> l(refMutex_);
  x = ref_x_;
  y = ref_y_;
  z = ref_z_;
  yaw = ref_yaw_;
  return true;
}

bool Autopilot::detectCollisions(const arp::kinematics::RobotState& x, double& x_ref, double& y_ref, double& z_ref, double& yaw_ref, double stepSize) {
  //get map limits
  int i_max, j_max, k_max;
  getMapLimits(i_max, j_max, k_max);

  int istart = std::round(x.t_WS[0]/0.1+double(i_max-1)/2.0);
  int jstart = std::round(x.t_WS[1]/0.1+double(j_max-1)/2.0);
  int kstart = std::round(x.t_WS[2]/0.1+double(k_max-1)/2.0);

  if(istart>=i_max || jstart>=j_max || kstart>=k_max || istart < 0 || jstart < 0 || kstart < 0 ) {
    std::cout << "Drone out of map limits, not allowing to move. Please change to manual mode and bring it back to the map area" << std::endl;
    x_ref = x.t_WS[0];
    y_ref = x.t_WS[1];
    z_ref = x.t_WS[2];
    return false;
  }

  int iref = std::round(x_ref/0.1+double(i_max-1)/2.0);
  int jref = std::round(y_ref/0.1+double(j_max-1)/2.0);
  int kref = std::round(z_ref/0.1+double(k_max-1)/2.0);

  if(iref>=i_max || jref>=j_max || kref>=k_max || iref < 0 || jref < 0 || kref < 0 ) {
    std::cout << "Goal out of map limits" << std::endl;
    x_ref = x.t_WS[0];
    y_ref = x.t_WS[1];
    z_ref = x.t_WS[2];
    return false;
  }

  //get direction of linear trajectory, unit vector, and number of steps according to given step size
  double dirX = x_ref - x.t_WS[0];
  double dirY = y_ref - x.t_WS[1];
  double dirZ = z_ref - x.t_WS[2];

  double distance = sqrt(dirX*dirX + dirY*dirY + dirZ*dirZ);
  dirX /= distance;
  dirY /= distance;
  dirZ /= distance;
  int numSteps = static_cast<int>(distance/stepSize);

  //check for collision against object or wall along trajectory und update reference values if necessary
  for(int c=0; c<=numSteps; ++c){
    //update current point along trajectory
    double currX = x.t_WS[0] + dirX*c*stepSize;
    double currY = x.t_WS[1] + dirY*c*stepSize;
    double currZ = x.t_WS[2] + dirZ*c*stepSize;
  
    //calculate corresponding grid value
    int i = std::round(currX/0.1+double(i_max-1)/2.0);
    int j = std::round(currY/0.1+double(j_max-1)/2.0);
    int k = std::round(currZ/0.1+double(k_max-1)/2.0);
  
    //check for collision in current point, update reference value and exit loop
    double occ;
    if(i>=i_max || j>=j_max || k>=k_max || double(occupancy_map.at<char>(i,j,k)) >= -0.5){

      do{
        currX = currX - dirX * 0.1;
        currY = currY - dirY * 0.1;
        currZ = currZ - dirZ * 0.1;

        int itemp = std::round(currX/0.1+double(i_max-1)/2.0);
        int jtemp = std::round(currY/0.1+double(j_max-1)/2.0);
        int ktemp = std::round(currZ/0.1+double(k_max-1)/2.0);
      
        if(itemp >= i_max || jtemp >= j_max || ktemp >= k_max || itemp<0 || jtemp<0 || ktemp<0) {
          std::cout << "Couldn't find any safe area in the vecinity of the safety point, use manual flight to unlock" << std::endl;
          x_ref = x.t_WS[0];
          y_ref = x.t_WS[1];
          z_ref = x.t_WS[2];
          return false;
        }
        occ = double(occupancy_map.at<char>(itemp, jtemp, ktemp));
      
      } while(occ > -0.9);

      x_ref = currX;
      y_ref = currY;
      z_ref = currZ;

      return true;
    }
  }
  return false;
}

bool Autopilot::getMapLimits(int& i, int& j, int& k) {
  i = occupancy_map.size[0];
  j = occupancy_map.size[1];
  k = occupancy_map.size[2];
  return true;
}

bool Autopilot::calculateForce(int i, int j, int k){
  //calculate forces
  double forceMagnX=0;
  double forceMagnY=0;
  double forceMagnZ=0;
  double forceMagn;
  double initOccupancy = -5;
  double maxOccupancy = -0.9;

  int i_max, j_max, k_max;
  double occ=0;
  getMapLimits(i_max, j_max, k_max);

  /*
  if(double(occupancy_map.at<char>(i+1,j,k)) - double(occupancy_map.at<char>(i,j,k)) > 0 || double(occupancy_map.at<char>(i-1,j,k)) - double(occupancy_map.at<char>(i,j,k)) > 0 || double(occupancy_map.at<char>(i,j,k)) > -0.9) {
    //ROS_INFO_STREAM("Collision in X direction");
    forceMagnX = 100; 
  } else {
    forceMagnX = 0;
  }
  if(double(occupancy_map.at<char>(i,j+1,k)) - double(occupancy_map.at<char>(i,j,k)) > 0 || double(occupancy_map.at<char>(i,j-1,k)) - double(occupancy_map.at<char>(i,j,k)) > 0 || double(occupancy_map.at<char>(i,j,k)) > -0.9) {
    //ROS_INFO_STREAM("Collision in Y direction");
    forceMagnY = 100; 
  } else {
    forceMagnY = 0;
  }
  if(double(occupancy_map.at<char>(i,j,k+1)) - double(occupancy_map.at<char>(i,j,k)) > 0 || double(occupancy_map.at<char>(i,j,k-1)) - double(occupancy_map.at<char>(i,j,k)) > 0 || double(occupancy_map.at<char>(i,j,k)) > -0.9) {
    //ROS_INFO_STREAM("Collision in Z direction");
    forceMagnZ = 100; 
  } else {
    forceMagnZ = 0;
  }
  */
  forceMagn = std::min(200/(maxOccupancy-initOccupancy) * (occupancy_map.at<char>(i,j,k) - initOccupancy), 200.0);
  

  //set forces
  Fc[0] = forceMagn * delta_x;
  Fc[1] = forceMagn * delta_y;
  Fc[2] = forceMagn * delta_z;
  return true;
}

bool Autopilot::publishForce(){
  feedback_.clear();
  copy(Fc, Fc+3, back_inserter(feedback_));
	std_msgs::Float64MultiArray feedback_msg;
  feedback_msg.data = feedback_;
  pubForce_.publish(feedback_msg);
  return true;
}

/// The callback from the estimator that sends control outputs to the drone
void Autopilot::controllerCallback(uint64_t timeMicroseconds,
                                  const arp::kinematics::RobotState& x)
{
  // only do anything here, if automatic
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    const double yaw = kinematics::yawAngle(x.q_WS);
    setPoseReference(x.t_WS[0], x.t_WS[1], x.t_WS[2], yaw);
    return;
  }
  //Collision avoidance
  // Get linear trajectory from interactive marker and use occupancyMap to check for collisions
  double x_ref, y_ref, z_ref, yaw_ref;
  getPoseReference(x_ref, y_ref, z_ref, yaw_ref);
  detectCollisions(x, x_ref, y_ref, z_ref, yaw_ref, 0.02);
  setPoseReference(x_ref, y_ref, z_ref, yaw_ref);
  int i_max, j_max, k_max;
  getMapLimits(i_max, j_max, k_max);

  //Calculate force according to occupancyValue and publish it
  int i = std::round(x.t_WS[0]/0.1+double(i_max-1)/2.0);
  int j = std::round(x.t_WS[1]/0.1+double(j_max-1)/2.0);
  int k = std::round(x.t_WS[2]/0.1+double(k_max-1)/2.0);
  calculateForce(i, j, k);
  publishForce();
  //Calculate error terms for controllers
  Eigen::Vector3d t_ref;
  t_ref[0]=x_ref;
  t_ref[1]=y_ref;
  t_ref[2]=z_ref;

  
  const Eigen::Matrix3d R_SW = x.q_WS.toRotationMatrix().inverse();

  Eigen::Vector3d t_error = R_SW*(t_ref - x.t_WS);
  double yaw_error = yaw_ref - kinematics::yawAngle(x.q_WS);

  if(yaw_error > M_PI){
    yaw_error -= 2*M_PI;
  }
  if(yaw_error < -M_PI){
    yaw_error += 2*M_PI;
  }

  Eigen::Vector3d t_error_dot = -R_SW*x.v_W;
  double yaw_error_dot = 0;
  

  // TODO: only enable when in flight
  int droneStatus = getDroneStatus();
  if (droneStatus==3 || droneStatus == 4){

  // TODO: get ros parameter
  double x_max, y_max, z_max, yaw_max;

  nh_->getParam("/ardrone_driver/euler_angle_max", x_max);
  nh_->getParam("/ardrone_driver/euler_angle_max", y_max);
  nh_->getParam("/ardrone_driver/control_vz_max", z_max);
  nh_->getParam("/ardrone_driver/control_yaw", yaw_max);

  // TODO: compute control output
  xController.setOutputLimits(-x_max, x_max);
  yController.setOutputLimits(-y_max, y_max);
  zController.setOutputLimits(-z_max*0.001, z_max*0.001);
  yawController.setOutputLimits(-yaw_max, yaw_max);

  double x_control = xController.control(timeMicroseconds, t_error[0], t_error_dot[0]);
  double y_control = yController.control(timeMicroseconds, t_error[1], t_error_dot[1]);
  double z_control = zController.control(timeMicroseconds, t_error[2], t_error_dot[2]);
  double yaw_control = yawController.control(timeMicroseconds, yaw_error, yaw_error_dot);

  // TODO: send to move
  //Normalize values to [-1,1]
  x_control = x_control / x_max;
  y_control = y_control / y_max;
  z_control = z_control / (z_max*0.001);
  yaw_control = yaw_control / yaw_max;
  move(x_control, y_control, z_control, yaw_control);
  }
}

}  // namespace arp

