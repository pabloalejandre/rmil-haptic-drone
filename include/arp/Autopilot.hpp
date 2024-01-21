/*
 * Autopilot.hpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_

#include <mutex>
#include <Eigen/Core>
#include <atomic>
#include <deque>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#include <arp/kinematics/Imu.hpp>

#include <arp/PidController.hpp>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

namespace arp {

/// \brief The autopilot highlevel interface for commanding the drone manually or automatically.
class Autopilot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Autopilot(ros::NodeHandle& nh);

  /// \brief These are reverse engineered AR Drone states.
  enum DroneStatus {
    Unknown = 0,
    Inited = 1,
    Landed = 2,
    Flying = 3,
    Hovering = 4,
    Test = 5, // ?
    TakingOff = 6,
    Flying2 = 7,
    Landing = 8,
    Looping = 9 // ?
  };

  /// \brief The current autopilot state.
  enum class AutopilotState {
    IDLE,
    FORWARD,
    LANDING,
    BACKWARD,
  };

  /// \brief Set to automatic control mode.
  void setManual();

  /// \brief Set to manual control mode.
  void setAutomatic();

  /// \brief Are we currently in automatic mode?;
  bool isAutomatic() { return isAutomatic_; }

  /// \brief Request flattrim calibration.
  /// \return True on success.
  /// \note This will only work when landed on ground.
  bool flattrimCalibrate();

  /// \brief Takeoff.
  /// \return True on success.
  /// \note This will only work when landed on ground.
  bool takeoff();

  /// \brief Land.
  /// \return True on success.
  /// \note This will only work when in flight.
  bool land();

  /// \brief Turn off all motors and reboot.
  /// \return True on success.
  /// \warning When in flight, this will let the drone drop down to the ground.
  bool estopReset();

  /// \brief Move the drone manually.
  /// @param[in] forward Forward tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] left Left tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] up Upward velocity [-1,...,1] scaling the maximum vertical speed ROS parameter.
  /// @param[in] rotateLeft Left yaw velocity [-1,...,1] scaling the maximum yaw rate ROS parameter.
  /// \return True on success.
  /// \note This will only do something when in manual mode and flying.
  bool manualMove(double forward, double left, double up, double rotateLeft);

  /// \brief Get the drone status.
  /// \return The status.
  DroneStatus getDroneStatus();
  /// \brief Get battery status.
  float getBatteryStatus() const;
  /// \brief Bool checks for flying status.
  bool isFlying() const;
  bool isLanded() const;
  
    /// \brief Move the drone automatically.
  /// @param[in] x World x position reference [m].
  /// @param[in] y World y position reference [m].
  /// @param[in] z World z position reference [m].
  /// @param[in] yaw Yaw angle reference [rad].
  /// \return True on success.
  /// \note  This will only do something when in automatic mode and flying.
  bool setPoseReference(double x, double y, double z, double yaw);

  /// \brief Get the pose reference.
  /// @param[out] x World x position reference [m].
  /// @param[out] y World y position reference [m].
  /// @param[out] z World z position reference [m].
  /// @param[out] yaw Yaw angle reference [rad].
  /// \return True on success.
  bool getPoseReference(double& x, double& y, double& z, double& yaw);
  
  /// \brief The callback from the estimator that sends control outputs to the drone
  /// \note  This will only do something when in automatic mode and flying.
  void controllerCallback(uint64_t timeMicroseconds,
                          const arp::kinematics::RobotState& x);

  /// Occupancy Map member for collision avoidance
  cv::Mat occupancy_map;

  bool getMapLimits(int& i, int& j, int& k);

  bool detectCollisions(const arp::kinematics::RobotState& x, double& x_ref, double& y_ref, double& z_ref, double& yaw_ref, double stepSize);
  
  bool calculateForce(int i, int j, int k);
  
  bool publishForce();

  double delta_x, delta_y, delta_z;
  double Fc[3];
  std::vector<double> feedback_;

 protected:
  /// \brief Move the drone.
  /// @param[in] forward Forward tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] left Left tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] up Upward velocity [-1,...,1] scaling the maximum vertical speed ROS parameter.
  /// @param[in] rotateLeft Left yaw velocity [-1,...,1] scaling the maximum yaw rate ROS parameter.
  /// \return True on success.
  bool move(double forward, double left, double up, double rotateLeft);

  /// \brief Obtain the last navdata package (callback).
  /// @param[out] navdata The navdata message.
  void navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg);

  ros::NodeHandle * nh_;  ///< ROS node handle.
  ros::Publisher pubReset_;  ///< The reset publisher -- use to reset the drone (e-stop).
  ros::Publisher pubTakeoff_;  ///< Publish to take-off the drone.
  ros::Publisher pubLand_;  ///< Publish to land the drone.
  ros::Publisher pubMove_;  ///< Publish to move the drone.
  ros::Publisher pubForce_; //Publish force to haptic device for force feedback.
  ros::ServiceClient srvFlattrim_;  ///< To request a flat trim calibration.
  ardrone_autonomy::Navdata lastNavdata_; ///< Store navdata as it comes in asynchronously.
  std::mutex navdataMutex_; ///< We need to lock navdata access due to asynchronous arrival.
  ros::Subscriber subNavdata_; ///< The subscriber for navdata.
  std::atomic<bool> isAutomatic_; ///< True, if in automatic control mode.
  
  double ref_x_ = 0.0; ///< World frame x position reference [m].
  double ref_y_ = 0.0; ///< World frame y position reference [m].
  double ref_z_ = 0.0; ///< World frame z position reference [m].
  double ref_yaw_ = 0.0; ///< World frame yaw reference [rad].
  std::mutex refMutex_; ///< We need to lock the reference access due to asynchronous arrival.

  PidController xController;
  PidController yController;
  PidController zController;
  PidController yawController;

};

} // namespace arp



#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_ */
