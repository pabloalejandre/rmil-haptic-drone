/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Dec 2, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/Transformation.hpp
 * @brief Header implementation file for the Transformation class.
 * @author Stefan Leutenegger
 */

/// \brief arp Main namespace of this package.
namespace arp {

/// \brief kinematics Namespace for kinematics functionality, i.e. transformations and stuff.
namespace kinematics {

__inline__ double sinc(double x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    static const double c_2 = 1.0 / 6.0;
    static const double c_4 = 1.0 / 120.0;
    static const double c_6 = 1.0 / 5040.0;
    const double x_2 = x * x;
    const double x_4 = x_2 * x_2;
    const double x_6 = x_2 * x_2 * x_2;
    return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
  }
}

__inline__ Eigen::Quaterniond deltaQ(const Eigen::Vector3d& dAlpha)
{
  Eigen::Vector4d dq;
  double halfnorm = 0.5 * dAlpha.template tail<3>().norm();
  dq.template head<3>() = sinc(halfnorm) * 0.5 * dAlpha.template tail<3>();
  dq[3] = cos(halfnorm);
  return Eigen::Quaterniond(dq);
}

// Right Jacobian, see Forster et al. RSS 2015 eqn. (8)
__inline__ Eigen::Matrix3d rightJacobian(const Eigen::Vector3d & PhiVec) {
  const double Phi = PhiVec.norm();
  Eigen::Matrix3d retMat = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d Phi_x = arp::kinematics::crossMx(PhiVec);
  const Eigen::Matrix3d Phi_x2 = Phi_x*Phi_x;
  if(Phi < 1.0e-4) {
    retMat += -0.5*Phi_x + 1.0/6.0*Phi_x2;
  } else {
    const double Phi2 = Phi*Phi;
    const double Phi3 = Phi2*Phi;
    retMat += -(1.0-cos(Phi))/(Phi2)*Phi_x + (Phi-sin(Phi))/Phi3*Phi_x2;
  }
  return retMat;
}

inline Transformation::Transformation(const Transformation & other)
    : parameters_(other.parameters_),
      t_(&parameters_[0]),
      q_(&parameters_[3]),
      R_(other.R_) {

}

inline Transformation::Transformation(Transformation && other)
    : parameters_(std::move(other.parameters_)),
      t_(&parameters_[0]),
      q_(&parameters_[3]),
      R_(std::move(other.R_)) {

}

inline Transformation::Transformation()
    : t_(&parameters_[0]),
      q_(&parameters_[3]),
      R_(Eigen::Matrix3d::Identity()) {
  t_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  q_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

inline Transformation::Transformation(const Eigen::Vector3d & t_AB,
                                      const Eigen::Quaterniond& q_AB)
    : t_(&parameters_[0]),
      q_(&parameters_[3]) {
  t_ = t_AB;
  q_ = q_AB.normalized();
  updateR();
}
inline Transformation::Transformation(const Eigen::Matrix4d & T_AB)
    : t_(&parameters_[0]),
      q_(&parameters_[3]),
      R_(T_AB.topLeftCorner<3, 3>()) {
  t_ = (T_AB.topRightCorner<3, 1>());
  q_ = (T_AB.topLeftCorner<3, 3>());
  assert(fabs(T_AB(3, 0)) < 1.0e-12);
  assert(fabs(T_AB(3, 1)) < 1.0e-12);
  assert(fabs(T_AB(3, 2)) < 1.0e-12);
  assert(fabs(T_AB(3, 3) - 1.0) < 1.0e-12);
}
inline Transformation::~Transformation() {

}

template<typename Derived_coeffs>
inline bool Transformation::setCoeffs(
    const Eigen::MatrixBase<Derived_coeffs> & coeffs) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_coeffs, 7);
  parameters_ = coeffs;
  updateR();
  return true;
}

// The underlying transformation
inline Eigen::Matrix4d Transformation::T() const {
  Eigen::Matrix4d T_ret;
  T_ret.topLeftCorner<3, 3>() = R_;
  T_ret.topRightCorner<3, 1>() = t_;
  T_ret.bottomLeftCorner<1, 3>().setZero();
  T_ret(3, 3) = 1.0;
  return T_ret;
}

// return the rotation matrix
inline const Eigen::Matrix3d & Transformation::R() const {
  return R_;
}

// return the translation vector
inline const Eigen::Map<Eigen::Vector3d> & Transformation::t() const {
  return t_;
}

inline const Eigen::Map<Eigen::Quaterniond> & Transformation::q() const {
  return q_;
}

inline Eigen::Matrix<double, 3, 4> Transformation::T3x4() const {
  Eigen::Matrix<double, 3, 4> T3x4_ret;
  T3x4_ret.topLeftCorner<3, 3>() = R_;
  T3x4_ret.topRightCorner<3, 1>() = t_;
  return T3x4_ret;
}
// Return a copy of the transformation inverted.
inline Transformation Transformation::inverse() const {
  return Transformation(-(R_.transpose() * t_), q_.inverse());
}

// Set this to a random transformation.
inline void Transformation::setRandom() {
  setRandom(1.0, M_PI);
}
// Set this to a random transformation with bounded rotation and translation.
inline void Transformation::setRandom(double translationMaxMeters,
                                      double rotationMaxRadians) {
  // Create a random unit-length axis.
  Eigen::Vector3d axis = rotationMaxRadians * Eigen::Vector3d::Random();
  // Create a random rotation angle in radians.
  Eigen::Vector3d t = translationMaxMeters * Eigen::Vector3d::Random();
  t_ = t;
  q_ = Eigen::AngleAxisd(axis.norm(), axis.normalized());
  updateR();
}

// Setters
inline void Transformation::set(const Eigen::Matrix4d & T_AB) {
  t_ = (T_AB.topRightCorner<3, 1>());
  q_ = (T_AB.topLeftCorner<3, 3>());
  updateR();
}
inline void Transformation::set(const Eigen::Vector3d & t_AB,
                                const Eigen::Quaternion<double> & q_AB) {
  t_ = t_AB;
  q_ = q_AB.normalized();
  updateR();
}
// Set this transformation to identity
inline void Transformation::setIdentity() {
  q_.setIdentity();
  t_.setZero();
  R_.setIdentity();
}

inline Transformation Transformation::Identity() {
  return Transformation();
}

// operator*
inline Transformation Transformation::operator*(
    const Transformation & rhs) const {
  return Transformation(R_ * rhs.t_ + t_, q_ * rhs.q_);
}
inline Eigen::Vector4d Transformation::operator*(
    const Eigen::Vector4d & rhs) const {
  const double s = rhs[3];
  Eigen::Vector4d retVec;
  retVec.head<3>() = R_ * rhs.head<3>() + t_ * s;
  retVec[3] = s;
  return retVec;
}

inline Transformation& Transformation::operator=(const Transformation & rhs) {
  parameters_ = rhs.parameters_;
  R_ = rhs.R_;
  t_ = Eigen::Map<Eigen::Vector3d>(&parameters_[0]);
  q_ = Eigen::Map<Eigen::Quaterniond>(&parameters_[3]);
  return *this;
}

inline void Transformation::updateR() {
  R_ = q_.toRotationMatrix();
}

// apply small update:
template<typename Derived_delta>
inline bool Transformation::oplus(
    const Eigen::MatrixBase<Derived_delta> & delta) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_delta, 6);
  t_ += delta.template head<3>();
  Eigen::Vector4d dq;
  double halfnorm = 0.5 * delta.template tail<3>().norm();
  dq.template head<3>() = sinc(halfnorm) * 0.5 * delta.template tail<3>();
  dq[3] = cos(halfnorm);
  q_ = (Eigen::Quaterniond(dq) * q_);
  q_.normalize();
  updateR();
  return true;
}

template<typename Derived_delta, typename Derived_jacobian>
inline bool Transformation::oplus(
    const Eigen::MatrixBase<Derived_delta> & delta,
    const Eigen::MatrixBase<Derived_jacobian> & jacobian) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_delta, 6);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 7, 6);
  if (!oplus(delta)) {
    return false;
  }
  return oplusJacobian(jacobian);
}

template<typename Derived_jacobian>
inline bool Transformation::oplusJacobian(
    const Eigen::MatrixBase<Derived_jacobian> & jacobian) const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 7, 6);
  Eigen::Matrix<double, 4, 3> S = Eigen::Matrix<double, 4, 3>::Zero();
  const_cast<Eigen::MatrixBase<Derived_jacobian>&>(jacobian).setZero();
  const_cast<Eigen::MatrixBase<Derived_jacobian>&>(jacobian)
      .template topLeftCorner<3, 3>().setIdentity();
  S(0, 0) = 0.5;
  S(1, 1) = 0.5;
  S(2, 2) = 0.5;
  const_cast<Eigen::MatrixBase<Derived_jacobian>&>(jacobian)
      .template bottomRightCorner<4, 3>() = arp::kinematics::oplus(q_) * S;
  return true;
}

template <typename Derived_jacobian>
inline bool Transformation::liftJacobian(const Eigen::MatrixBase<Derived_jacobian> & jacobian) const
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 6, 7);
  const_cast<Eigen::MatrixBase<Derived_jacobian>&>(jacobian).setZero();
  const_cast<Eigen::MatrixBase<Derived_jacobian>&>(jacobian).template topLeftCorner<3,3>()
      = Eigen::Matrix3d::Identity();
  const_cast<Eigen::MatrixBase<Derived_jacobian>&>(jacobian).template bottomRightCorner<3,4>()
      = 2*arp::kinematics::oplus(q_.inverse()).template topLeftCorner<3,4>();
  return true;
}

}  // namespace kinematics
}  // namespace arp
