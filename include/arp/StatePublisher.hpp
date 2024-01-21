/*
 * StatePublisher.hpp
 *
 *  Created on: 8 Feb 2017
 *      Author: sleutene
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_STATEPUBLISHER_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_STATEPUBLISHER_HPP_

#include <Eigen/Core>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <arp/kinematics/Imu.hpp>

namespace arp {

class StatePublisher
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StatePublisher(ros::NodeHandle& nh)
    : nh_(&nh)
  {
    // for rviz
    pubPose_ = nh_->advertise<geometry_msgs::PoseStamped>(
                      "ardrone/vi_ekf_pose", 1);
  }

  void publish(uint64_t timeMicroseconds,
                    const arp::kinematics::RobotState& x)
  {
    arp::kinematics::Transformation T_WS(x.t_WS, x.q_WS);
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = "world";
    // rviz can't handle original timestamps. won't matter, it's visualisation only...
    poseMsg.header.stamp = ros::Time::now();

    // fill orientation
    Eigen::Quaterniond q = T_WS.q();
    poseMsg.pose.orientation.x = q.x();
    poseMsg.pose.orientation.y = q.y();
    poseMsg.pose.orientation.z = q.z();
    poseMsg.pose.orientation.w = q.w();

    // fill position
    Eigen::Vector3d t = T_WS.t();
    poseMsg.pose.position.x = t[0];
    poseMsg.pose.position.y = t[1];
    poseMsg.pose.position.z = t[2];

    //publish stamped transform
    pubPose_.publish(poseMsg);
  }
protected:
  ros::NodeHandle * nh_;  ///< ROS node handle.
  ros::Publisher pubPose_;  ///< The publisher for the transform.
};

}

#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_STATEPUBLISHER_HPP_ */

