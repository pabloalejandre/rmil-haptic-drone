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
 *  Created on: Feb 3, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/RadialTangentialDistortion.hpp
 * @brief Header implementation file for the RadialTangentialDistortion class.
 * @author Stefan Leutenegger
 */


#include <Eigen/LU>
#include <iostream>
#include <stdexcept>

/// \brief arp Main namespace of this package.
namespace arp {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

// The default constructor with all zero ki
RadialTangentialDistortion::RadialTangentialDistortion()
    : k1_(0.0),
      k2_(0.0),
      p1_(0.0),
      p2_(0.0)
{
}

// Constructor initialising ki
RadialTangentialDistortion::RadialTangentialDistortion(double k1, double k2,
                                                       double p1, double p2)
{
  k1_ = k1;
  k2_ = k2;
  p1_ = p1;
  p2_ = p2;
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted,
    Eigen::Vector2d * pointDistorted) const
{
  double r2 = pointUndistorted.squaredNorm();
  double x1 = pointUndistorted[0];
  double x2 = pointUndistorted[1];
  double c = 1 + k1_ * r2 + k2_ * pow(r2, 2);
  *pointDistorted << c * x1 + 2 * p1_ * x1 * x2 + p2_ * (r2 + 2 * pow(x1, 2)), 
                     c * x2 + p1_ * (r2 + 2 * pow(x2, 2) + 2 * p2_ * x1 * x2);
  return true;
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted, Eigen::Vector2d * pointDistorted,
    Eigen::Matrix2d * pointJacobian) const
{
  if(!pointJacobian) return false;

  bool success = distort(pointUndistorted, pointDistorted);
  if(!success) 
  {
    return false;
  }

//   double r2 = pointUndistorted.squaredNorm();
//   double x1 = pointUndistorted[0];
//   double x2 = pointUndistorted[1];  
//   double c = 1 + k1_ * r2 + k2_ * pow(r2, 2);
  
//   // (x^2 + y^2)^2 = x^4 + 2 * x^2 * y^2 + y^4
//   // dc1 = 2 * k1_ * x1 + k2_ * (4 * pow(x1, 3) + 4 * pow(x2, 2) * x1)
//   // dc1 = x1 * (2 * k1_ + k2_ * 4 * r2);
//   // dc2 = 2 * k1_ * x2 + k2_ * (4 * pow(x2, 3) + 4 * pow(x1, 2) * x2);
//   // dc2 = x2 * (2 * k1_ + k2_ * 4 * r2);
//   double dc = 2 * k1_ + k2_ * 4 * r2;
//   double J11 = c + dc * pow(x1, 2) + 2 * p1_ * x2 + p2_ * 6 * x1;
//   double J22 = c + dc * pow(x2, 2) + 2 * p2_ * x1 + p1_ * 6 * x2;
//   double J12 = dc * x1 * x2 + 2 * p1_ * x1 + 2 * p2_ * x2;

//   *pointJacobian << J11, J12,
//                     J12, J22;
//   return true;

  const double dx = 1.0e-7;
  Eigen::Matrix2d& Jn = *pointJacobian;
  for(int i = 0; i < 2; i++) 
  {
    Eigen::Vector2d pointUndistorted_p = pointUndistorted;
    pointUndistorted_p[i] += dx;
    Eigen::Vector2d pointUndistorted_n = pointUndistorted;
    pointUndistorted_n[i] -= dx;

    Eigen::Vector2d pointDistorted_p;
    Eigen::Vector2d pointDistorted_n;
    distort(pointUndistorted_p, &pointDistorted_p);
    distort(pointUndistorted_n, &pointDistorted_n);    
    Jn(0, i) = (pointDistorted_p[0] - pointDistorted_n[0]) / (2 * dx);
    Jn(1, i) = (pointDistorted_p[1] - pointDistorted_n[1]) / (2 * dx);
  }

  return true;
}

bool RadialTangentialDistortion::undistort(
    const Eigen::Vector2d & pointDistorted,
    Eigen::Vector2d * pointUndistorted) const
{
  // this is expensive: we solve with Gauss-Newton...
  Eigen::Vector2d x_bar = pointDistorted; // initialise at distorted point
  const int n = 5;  // just 5 iterations max.
  Eigen::Matrix2d E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; i++) {

    Eigen::Vector2d x_tmp;

    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2d e(pointDistorted - x_tmp);
    Eigen::Matrix2d E2 = (E.transpose() * E);
    Eigen::Vector2d du = E2.inverse() * E.transpose() * e;

    x_bar += du;

    const double chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-15) {
      success = true;
      break;
    }

  }
  *pointUndistorted = x_bar;

  return success;
}

}  // namespace cameras
}  // namespace arp
