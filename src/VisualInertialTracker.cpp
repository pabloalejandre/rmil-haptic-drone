/*
 * VisualInertialTracker.cpp
 *
 *  Created on: 8 Dec 2016
 *      Author: sleutene
 */

#include <ros/ros.h>

#include <arp/VisualInertialTracker.hpp>

namespace arp {

VisualInertialTracker::VisualInertialTracker()
{
  processingThread_ = std::thread(&VisualInertialTracker::processingLoop, this);
  controllerThread_ = std::thread(&VisualInertialTracker::controllerLoop, this);
  visualisationThread_ = std::thread(&VisualInertialTracker::visualisationLoop, this);
}

VisualInertialTracker::~VisualInertialTracker()
{
  imuMeasurementQueue_.Shutdown();
  cameraMeasurementQueue_.Shutdown();
  cameraVisualisationQueue_.Shutdown();
  controllerQueue_.Shutdown();
  visualisationQueue_.Shutdown();
  processingThread_.join();
  controllerThread_.join();
  visualisationThread_.join();
}

void VisualInertialTracker::addImage(uint64_t timestampMicroseconds,
                                     const cv::Mat& image)
{
  CameraMeasurement cameraMeasurement;
  cameraMeasurement.timestampMicroseconds = timestampMicroseconds;
  cameraMeasurement.image = image;
  cameraMeasurementQueue_.PushNonBlockingDroppingIfFull(cameraMeasurement,2);
}

void VisualInertialTracker::addImuMeasurement(uint64_t timestampMicroseconds,
                                              const Eigen::Vector3d& omega_S,
                                              const Eigen::Vector3d& acc_S)
{
  kinematics::ImuMeasurement imuMeasurement;
  imuMeasurement.timestampMicroseconds = timestampMicroseconds;
  imuMeasurement.omega_S = omega_S;
  imuMeasurement.acc_S = acc_S;
  imuMeasurementQueue_.PushNonBlocking(imuMeasurement);
}

void VisualInertialTracker::processingLoop()
{
  // never end
  for (;;) {

    // get camera measurement
    CameraMeasurement cameraMeasurement;
    if (!cameraMeasurementQueue_.PopBlocking(&cameraMeasurement)) {
      return;
    } else {

      // get IMU measurement
      kinematics::ImuMeasurement imuMeasurement;
      uint64_t t = 0;

      while (t<cameraMeasurement.timestampMicroseconds) {
        if(!imuMeasurementQueue_.PopBlocking(&imuMeasurement)){
          return;
        }
        // feed to estimator (propagation)
        estimator_->addImuMeasurement(imuMeasurement.timestampMicroseconds,
                                           imuMeasurement.omega_S,
                                           imuMeasurement.acc_S);
        // publish to controller (high rate)
        kinematics::RobotState x;
        t = imuMeasurement.timestampMicroseconds;
        if (estimator_->getState(t, x)) {
          if(controllerCallback_) {
            StateEstimate estimate;
            estimate.timestampMicroseconds = t;
            estimate.state = x;
            controllerQueue_.PushNonBlocking(estimate);
          }
        }
      }

      if (cameraMeasurement.timestampMicroseconds > t) {
        ROS_WARN_STREAM("bad timestamp");
      }
      DetectionVec detections;
      kinematics::Transformation T_WS, T_CW;
      cv::Mat visualisationImage = cameraMeasurement.image.clone();
      kinematics::RobotState xi;
      bool successGetState = estimator_->getState(cameraMeasurement.timestampMicroseconds, xi);
      Eigen::Vector3d dir(0,1,0);
      if(successGetState) {
        T_WS = kinematics::Transformation(xi.t_WS, xi.q_WS);
        dir = estimator_->T_SC().inverse().R()*T_WS.inverse().R()*Eigen::Vector3d(0,0,-1);
        T_CW = estimator_->T_SC().inverse() * T_WS.inverse();
      }
      bool needsReInitialisation = true;
      if(successGetState && estimator_->isInitialised()) {
        needsReInitialisation = false;
      }
      bool ransacSuccess = frontend_->detectAndMatch(
          cameraMeasurement.image, dir, detections, T_CW, visualisationImage, needsReInitialisation);
      if (detections.size() > 0) {
        // feed to estimator (measurement update)
        if (estimator_->isInitialised() && fusionEnabled_) {
          estimator_->addKeypointMeasurements(
              cameraMeasurement.timestampMicroseconds, detections);
        } else if (ransacSuccess){
          // use initialisation from RANSAC to init state
          T_WS = T_CW.inverse() * estimator_->T_SC().inverse();
          kinematics::RobotState x;
          Eigen::Matrix<double, 15, 15> P =
              Eigen::Matrix<double, 15, 15>::Identity();
          x.t_WS = T_WS.t();
          x.q_WS = T_WS.q();
          x.v_W.setZero();
          x.b_g.setZero();
          x.b_a.setZero();
          // and init covariance
          P.block<3, 3>(0, 0) *= 0.0001;  // 1 cm
          P.block<3, 3>(3, 3) *= (0.5 / 60.0) * (0.5 / 60.0);  // 0.5 degree
          P.block<3, 3>(6, 6) *= 0.01 * 0.01;  // 10 cm/s
          P.block<3, 3>(9, 9) *= (1.0 / 60.0) * (1.0 / 60.0);  // 1 degree/sec
          P.block<3, 3>(12, 12) *= 0.1;  // 1 m/s^2
          ROS_INFO_STREAM("initialising ViEKF state");
          estimator_->initialiseState(cameraMeasurement.timestampMicroseconds, x, P);
        }
      }

      // publish for visualisation
      CameraMeasurement visualisation;
      visualisation.timestampMicroseconds = cameraMeasurement.timestampMicroseconds;
      visualisation.image = visualisationImage;
      cameraVisualisationQueue_.PushNonBlockingDroppingIfFull(visualisation, 1);
      kinematics::RobotState x;
      if (estimator_->getState(cameraMeasurement.timestampMicroseconds, x)) {
        if(visualisationCallback_) {
          StateEstimate estimate;
          estimate.timestampMicroseconds = cameraMeasurement.timestampMicroseconds;
          estimate.state = x;
          visualisationQueue_.PushNonBlocking(estimate);
        }
      }
    }
  }
}

void VisualInertialTracker::controllerLoop()
{
  // never end
  for (;;) {
    // get estimator output
    StateEstimate stateEstimate;
    if (!controllerQueue_.PopBlocking(&stateEstimate)) {
      return;
    } else {
      if (controllerCallback_) {
        controllerCallback_(stateEstimate.timestampMicroseconds, stateEstimate.state);
      }
    }
  }
}

void VisualInertialTracker::visualisationLoop()
{
  // never end
  for (;;) {
    // get estimator output
    StateEstimate stateEstimate;
        if (!visualisationQueue_.PopBlocking(&stateEstimate)) {
      return;
    } else {
      if (visualisationCallback_) {
        visualisationCallback_(stateEstimate.timestampMicroseconds, stateEstimate.state);
      }
    }
  }
}

bool VisualInertialTracker::getLastVisualisationImage(cv::Mat & visualisationImage) {
  CameraMeasurement visualisation;
  bool success = cameraVisualisationQueue_.PopNonBlocking(&visualisation);
  visualisationImage = visualisation.image;
  return success;
}

}  // namespace arp

