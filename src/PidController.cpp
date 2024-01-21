/*
 * PidController.cpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#include <arp/PidController.hpp>
#include <stdexcept>

namespace arp {

// Set the controller parameters
void PidController::setParameters(const Parameters & parameters)
{
  parameters_ = parameters;
}

// Implements the controller as u(t)=c(e(t))
double PidController::control(uint64_t timestampMicroseconds, double e,
                              double e_dot)
{
  // TODO: implement...

  uint64_t deltaT_us = timestampMicroseconds-lastTimestampMicroseconds_;
  double deltaT = ((double) deltaT_us )/1000000.0;

  if(deltaT > 0.1){
      deltaT = 0.1;
  } 
  double p_part = parameters_.k_p * e;
  double d_part = parameters_.k_d * e_dot;
  double i_part = parameters_.k_i * integratedError_;
  
  double output = p_part + d_part + i_part;

  if(output > maxOutput_){
    output = maxOutput_;    
  }
  else if(output < minOutput_){
    output = minOutput_;    
  }
  else{
    integratedError_ += e * deltaT;
  }

  return output;
}

// Reset the integrator to zero again.
void PidController::setOutputLimits(double minOutput, double maxOutput)
{
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

/// \brief
void PidController::resetIntegrator()
{
  integratedError_ = 0.0;
}

}  // namespace arp
