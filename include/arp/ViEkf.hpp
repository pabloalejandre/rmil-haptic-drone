/*
 * ViEkf.hpp
 *
 *  Created on: 20 Nov 2015
 *      Author: sleutene
 */

#ifndef ARP_VIEKF_HPP_
#define ARP_VIEKF_HPP_

#include <map>
#include <deque>
#include <Eigen/StdVector>
#include <arp/kinematics/Transformation.hpp>
#include <arp/kinematics/Imu.hpp>
#include <arp/cameras/CameraBase.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>

namespace arp {

class ViEkf
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ViEkf();
  ~ViEkf() {}

  /// \brief Set the pose of the camera relative to the IMU.
  /// \param[in] T_SC The pose of the camera relative to the IMU (extrinsics).
  bool setCameraExtrinsics(const kinematics::Transformation & T_SC);

  /// \brief Set the intrinsics of the camera, i.e. the camera model
  /// \param[in] cameraModel The camera model (intrinsics).
  bool setCameraIntrinsics(
      const cameras::PinholeCamera<cameras::RadialTangentialDistortion> & cameraModel);

  /// \brief Set IMU noise parameters.
  /// \param[in] sigma_c_gyr The continuous-time noise density of the gyros [rad/(s*sqrt(Hz))].
  /// \param[in] sigma_c_acc The continuous-time noise density of the accelerometers [m/(s^2*sqrt(Hz))].
  /// \param[in] sigma_c_gw The gyro bias drift continuous-time noise density [rad/(s^2*sqrt(Hz))].
  /// \param[in] sigma_c_aw The accelerometer bias drift continuous-time noise density [m/(s^3*sqrt(Hz))].
  void setImuNoiseParameters(double sigma_c_gyr, double sigma_c_acc, double sigma_c_gw, double sigma_c_aw);

  /// \brief Set measurement noise parameter.
  /// \param[in] sigma_imagePoint The keypoint detection noise [pixel]
  void setDetectorNoiseParameter(double sigma_imagePoint);

  /// \brief Initialise the states
  /// \param[in] timestampMicroseconds The IMU measurement timestamp [usec].
  /// \param[in] x Set the state estimate.
  /// \param[in] P Set the covariance matrix.
  bool initialiseState(uint64_t timestampMicroseconds,
                       const kinematics::RobotState & x,
                       const Eigen::Matrix<double, 15, 15> & P);

  /// \brief Has this EKF been initialised?
  bool isInitialised() const;

  /// \brief Get the states.
  /// \param[in] timestampMicroseconds Time for which you require the state.
  /// \param[out] x The current state estimate.
  /// \param[out] P The current covariance matrix.
  bool getState(uint64_t timestampMicroseconds, kinematics::RobotState & x,
                Eigen::Matrix<double, 15, 15> * P = nullptr);

  /// \brief Obtain the camera extrinsics.
  /// \return T_SC The camera pose.
  kinematics::Transformation T_SC() const {
    return T_SC_;
  }

  /// \brief Takes the IMU measurements but makes sure that the state is not immediately predicted
  ///        in order to synchronise with the updates
  /// \param[in] timestampMicroseconds The IMU measurement timestamp [usec].
  /// \param[in] gyroscopeMeasurements The rotation rates from the gyros [rad/s].
  /// \param[in] accelerometerMeasurements The acceleration from the accelerometers [m/s^2].
  bool addImuMeasurement(uint64_t timestampMicroseconds,
               const Eigen::Vector3d & gyroscopeMeasurements,
               const Eigen::Vector3d & accelerometerMeasurements);

  /// \brief Pass a set of keypoint measurements to trigger an update.
  /// \param[in] timestampMicroseconds The IMU measurement timestamp [usec].
  /// \param[in] detectoinVec The measured 2d pts in the image / corresponding 3d pts.
  bool addKeypointMeasurements(uint64_t timestampMicroseconds,
              const DetectionVec & detectionVec);

  
  kinematics::RobotState x_;  ///< The state estimates.

 protected:

  /// \brief The EKF prediction.
  /// \param[in] from_timestampMicroseconds The time to predict from.
  /// \param[in] to_timestampMicroseconds The time to predict to.
  bool predict(uint64_t from_timestampMicroseconds, uint64_t to_timestampMicroseconds);

  /// \brief The EKF update.
  /// \param[in] detection The measured 2d pt and the corresponding 3d landmark.
  bool update(const Detection & detection);

  // camera parameters
  kinematics::Transformation T_SC_;  ///< The pose of the camera relative to the IMU (extrinsics).
  cameras::PinholeCamera<cameras::RadialTangentialDistortion> cameraModel_;  ///< The camera model (intrinsics).

  // noise parameters
  double sigma_c_gyr_ = 1.0e-2;//1.0e-2;  ///< The continuous-time noise density of the gyros [rad/(s*sqrt(Hz))].
  double sigma_c_acc_ = 1.0e-1; //1.0e-1;  ///< The continuous-time noise density of the accelerometers [m/(s^2*sqrt(Hz))].
  double sigma_c_gw_ = 3.0e-5;//3.0e-5;  ///< The gyro bias drift continuous-time noise density [rad/(s^2*sqrt(Hz))].
  double sigma_c_aw_ = 2.0e-4;//2.0e-4;  ///< The accelerometer bias drift continuous-time noise density [m/(s^3*sqrt(Hz))].
  double sigma_imagePoint_ = 1.0;  ///< The corner detection noise [pixel].

  // the internal state
  
  Eigen::Matrix<double, 15, 15> P_;  ///< The covariance matrix.

  // propagated state for up-to-date publishing
  kinematics::RobotState x_propagated_; ///< The propagated state estimates (for access only).
  uint64_t timestampPropagatedMicrosec_ = 0; ///< Remembers when last propagated state (for access only) was made [usec]

  // buffer for the IMU measurements
  std::deque<kinematics::ImuMeasurement> imuMeasurementDeque_; ///< A deque that buffers received IMU measurements.
  uint64_t timestampLastUpdateMicrosec_ = 0; ///< Remembers when last update was made [usec]
  bool lastTimeReject_=false;
  int rejections_=0;
};

}  // namespace arp

#endif /* ARP_VIEKF_HPP_ */
