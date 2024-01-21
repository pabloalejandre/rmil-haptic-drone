/*
 * Imu.cpp
 *
 *  Created on: 8 Feb 2017
 *      Author: sleutene
 */

#include <arp/kinematics/Imu.hpp>
#include <iostream>

namespace arp {
namespace kinematics {

#define g Eigen::Vector3d(0, 0, -9.81)  


TangentState fc(const RobotState& x, const ImuMeasurement& z) {
  TangentState dchi;

  dchi.t_WS = x.v_W;
  dchi.alpha_WS = x.q_WS.toRotationMatrix() * (z.omega_S - x.b_g);
  dchi.v_W = x.q_WS.toRotationMatrix() * (z.acc_S - x.b_a) + g;
  dchi.b_g = Eigen::Vector3d::Zero();  // expected (thus unnoised) state
  dchi.b_a = Eigen::Vector3d::Zero();  // expected (thus unnoised) state
  
  return dchi;
}

Eigen::Matrix<double, 15, 15> Fc(const RobotState& x, const ImuMeasurement& z) {
  Eigen::Matrix<double, 3, 3> zeros = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> eye = Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 3> R_WS = x.q_WS.toRotationMatrix();

  Eigen::Matrix<double, 3, 3> dalpha = - crossMx(R_WS * (z.omega_S - x.b_g));
  Eigen::Matrix<double, 3, 3> dv = - crossMx(R_WS * (z.acc_S - x.b_a));

  Eigen::Matrix<double, 15, 15> F;
  F << zeros, zeros, eye, zeros, zeros, 
       // zeros, dalpha, zeros, -R_WS, zeros,
       zeros, zeros, zeros, -R_WS, zeros,
       zeros, dv, zeros, zeros, -R_WS,
       zeros, zeros, zeros, zeros, zeros,
       zeros, zeros, zeros, zeros, zeros;

  return F;
}


bool Imu::stateTransition(const RobotState & state_k_minus_1,
                          const ImuMeasurement & z_k_minus_1,
                          const ImuMeasurement & z_k, 
                          RobotState & state_k,
                          ImuKinematicsJacobian* jacobian)
{
  const double dt = double(z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;
  if (dt < 1.0e-12 || dt > 0.05) {
    // for safety, we assign reasonable values here.
    state_k = state_k_minus_1;
    if(jacobian) {
      jacobian->setIdentity();
    }
    return false;  // negative, no or too large time increments not permitted
  }

  TangentState dx1 = fc(state_k_minus_1, z_k_minus_1) * dt;
  RobotState state_km1_plus_dx1 = arp::kinematics::RobotState::computeTangentUpdate(state_k_minus_1, dx1);
  TangentState dx2 = fc(state_km1_plus_dx1, z_k) * dt;
  
  TangentState dchi = (dx1 + dx2) * 0.5;
  state_k = arp::kinematics::RobotState::computeTangentUpdate(state_k_minus_1, dchi);

  if(jacobian) {
    const Eigen::Matrix<double, 15, 15> F_km1 = Fc(state_k_minus_1, z_k_minus_1);
    const Eigen::Matrix<double, 15, 15> F_k = Fc(state_km1_plus_dx1, z_k);
    const Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    *jacobian = I + dt / 2.0 * F_km1 + dt / 2.0 * (F_k * (I + dt * F_km1));

    (*jacobian).block<3,3> (3,3) = Eigen::Matrix3d::Identity();
    (*jacobian).block<3,3> (3,9) = - 0.5 * dt * (
        state_k.q_WS.toRotationMatrix() + state_k_minus_1.q_WS.toRotationMatrix());
    (*jacobian).block<3,3> (6,3) = -0.5 * dt * crossMx(
        state_k_minus_1.q_WS.toRotationMatrix()* (z_k_minus_1.acc_S - state_k_minus_1.b_a) 
        + state_k.q_WS.toRotationMatrix()* (z_k.acc_S - state_k.b_a));
  }

  return true;
}

}
}  // namespace arp
