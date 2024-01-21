#ifndef ARP_KINEMATICS_TRANSFORMATION_HPP
#define ARP_KINEMATICS_TRANSFORMATION_HPP

#include <Eigen/Core>
#include <arp/kinematics/Transformation.hpp>
#include <arp/kinematics/operators.hpp>
#include <iostream>

namespace arp {
namespace kinematics {

struct TangentState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d t_WS;  ///< The position relative to the W frame.
  Eigen::Vector3d alpha_WS;  ///< The tangent quaternion of rotation W-S.
  Eigen::Vector3d v_W;  ///< The velocity expressed in W frame.
  Eigen::Vector3d b_g;  ///< The gyro bias.
  Eigen::Vector3d b_a;  ///< The accelerometer bias.

  TangentState operator*(const double t) {
    TangentState x;
    x.t_WS = t * t_WS;
    x.alpha_WS = t * alpha_WS;
    x.v_W = t * v_W;
    x.b_g = t * b_g;
    x.b_a = t * b_a;
    return x;
  }

  friend TangentState operator+(const TangentState& x1, const TangentState& x2) {
    TangentState x;
    x.t_WS = x1.t_WS + x2.t_WS;
    x.alpha_WS = x1.alpha_WS + x2.alpha_WS;
    x.v_W = x1.v_W + x2.v_W;
    x.b_g = x1.b_g + x2.b_g;
    x.b_a = x1.b_a + x2.b_a;
    return x;
  }

};

struct RobotState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d t_WS;  ///< The position relative to the W frame.
  Eigen::Quaterniond q_WS;  ///< The quaternion of rotation W-S.
  Eigen::Vector3d v_W;  ///< The velocity expressed in W frame.
  Eigen::Vector3d b_g;  ///< The gyro bias.
  Eigen::Vector3d b_a;  ///< The accelerometer bias.

  friend RobotState operator+(const RobotState& x1, const RobotState& x2) {
    RobotState x;
    x.t_WS = x1.t_WS + x2.t_WS;
    x.q_WS = arp::kinematics::plus(x1.q_WS) * x2.q_WS.coeffs();
    // x.q_WS.normalize();  // quaternion "addition" preservers unit norm
    x.v_W = x1.v_W + x2.v_W;
    x.b_g = x1.b_g + x2.b_g;
    x.b_a = x1.b_a + x2.b_a;
    return x;
  }

  static RobotState computeTangentUpdate(const RobotState& x1, const TangentState& delta) {
    RobotState x;
    x.t_WS = x1.t_WS + delta.t_WS;
    x.q_WS = arp::kinematics::plus(arp::kinematics::deltaQ(delta.alpha_WS)) * x1.q_WS.coeffs();
    // x.q_WS.normalize();  // quaternion "addition" preservers unit norm
    x.v_W = x1.v_W + delta.v_W;
    x.b_g = x1.b_g + delta.b_g;
    x.b_a = x1.b_a + delta.b_a;
    return x;
  }

  static RobotState computeTangentUpdate(const RobotState& x1, const Eigen::Matrix<double, 15, 1>& delta) {
    RobotState x;
    x.t_WS = x1.t_WS + delta.segment<3>(0);
    x.q_WS = arp::kinematics::plus(arp::kinematics::deltaQ(delta.segment<3>(3))) * x1.q_WS.coeffs();
    // x.q_WS.normalize();  // quaternion "addition" preservers unit norm
    x.v_W = x1.v_W + delta.segment<3>(6);
    x.b_g = x1.b_g + delta.segment<3>(9);
    x.b_a = x1.b_a + delta.segment<3>(12);
    return x;
  }

  void print() {
    std::cout << t_WS.transpose() << std::endl
              << q_WS.coeffs().transpose() << std::endl
              << v_W.transpose() << std::endl << std::endl;
  }

};

typedef Eigen::Matrix<double,15,15> ImuKinematicsJacobian;

struct ImuMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t timestampMicroseconds; ///< Timestamp of the measurement in [usec].
  Eigen::Vector3d omega_S; ///< The gyro reading omega_tilde_S [rad/s].
  Eigen::Vector3d acc_S; ///< The accelerometer reading a_tilde_S [rm/s^2/s].
};

class Imu
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \brief Default constructor, does nothing
  Imu()
  {
  }

  /// \brief Implements the discrete-time nonlinear IMU kinematics
  ///        x_k = f(x_k_minus_1,z_imu). It will also compute the
  ///        linearised kinematics in the sense
  ///        delta_chi_k \approx F_k * delta_chi_k_minus_1.
  /// @param[in]  state_k_minus_1  Last state.
  /// @param[in]  z_k_minus_1      Last IMU measurement.
  /// @param[in]  z_k              Current IMU measurement.
  /// @param[out] state_k          Current state computed.
  /// @param[out] jacobian         state transition matrix F_k.
  static bool stateTransition(const RobotState & state_k_minus_1,
                              const ImuMeasurement & z_k_minus_1,
                              const ImuMeasurement & z_k, 
                              RobotState & state_k,
                              ImuKinematicsJacobian* jacobian = nullptr);
};

}
} // namespace arp

#endif // ARP_KINEMATICS_TRANSFORMATION_HPP
