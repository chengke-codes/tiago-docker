///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Wim Meeussen

#ifndef HARDWARE_INTERFACE_ACTUATOR_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_ACTUATOR_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace hardware_interface
{

/** A handle used to read the state of a single actuator. */
class ActuatorStateHandle
{
public:
  ActuatorStateHandle() : name_(), pos_(0), vel_(0), eff_(0), absolute_pos_(0), torque_sensor_(0) {}

  /**
   * \param name The name of the actuator
   * \param pos A pointer to the storage for this actuator's position
   * \param vel A pointer to the storage for this actuator's velocity
   * \param eff A pointer to the storage for this actuator's effort (force or torque)
   */

  ActuatorStateHandle(const std::string& name, const double* pos, const double* vel,
                      const double* eff, const double* absolute_pos = nullptr,
                      const double* torque_sensor = nullptr,
                      const std::vector<std::vector<double>>* pid_gains = nullptr,
                      const std::vector<double>* ff_term = nullptr)
    : name_(name)
    , pos_(pos)
    , vel_(vel)
    , eff_(eff)
    , absolute_pos_(absolute_pos)
    , torque_sensor_(torque_sensor)
    , pid_gains_(pid_gains)
    , ff_term_(ff_term)
  {
    if (!pos)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Position data pointer is null.");
    }
    if (!vel)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Velocity data pointer is null.");
    }
    if (!eff)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Effort data pointer is null.");
    }
    if(pid_gains_ && (*pid_gains_).size() != 3)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name +
                                       "'. The parsed PID gains pointer is not of size 3.");
    }

  }

  std::string getName() const {return name_;}
  double getPosition()  const {assert(pos_); return *pos_;}
  double getVelocity()  const {assert(vel_); return *vel_;}
  double getEffort()    const {assert(eff_); return *eff_;}
  std::vector<double> getFFTerm()    const {assert(ff_term_); return *ff_term_;}
  std::vector<std::vector<double>> getPIDGains() const
  {
    assert(pid_gains_);
    return *pid_gains_;
  }
  double getAbsolutePosition() const {
    assert(absolute_pos_);
    if(!absolute_pos_){
     throw std::runtime_error("Actuator does not support absolute encoders");
    }
    return *absolute_pos_;
  }
  double getTorqueSensor() const {
    assert(torque_sensor_);
    if(!torque_sensor_){
      throw std::runtime_error("Actuator does not support torque sensors");
    }
    return *torque_sensor_;
  }

  const double* getPositionPtr() const {return pos_;}
  const double* getVelocityPtr() const {return vel_;}
  const double* getEffortPtr()   const {return eff_;}
  const std::vector<std::vector<double>>* getPIDGainsPtr() const {return pid_gains_;}
  const std::vector<double>* getFFTermPtr()   const {return ff_term_;}
  const double* getAbsolutePositionPtr() const {
    if(!absolute_pos_){
     throw std::runtime_error("Actuator does not support absolute encoders");
    }
    return absolute_pos_;
  }
  const double* getTorqueSensorPtr()   const {
    if(!torque_sensor_){
      throw std::runtime_error("Actuator does not support torque sensors");
    }
    return torque_sensor_;
  }

  bool hasAbsolutePosition(){
    if(!absolute_pos_){
     return false;
    }
    else{
      return true;
    }
  }

  bool hasTorqueSensor(){
    if(!torque_sensor_){
     return false;
    }
    else{
      return true;
    }
  }

private:
  std::string name_;
  const double* pos_;
  const double* vel_;
  const double* eff_;
  const double* absolute_pos_;
  const double* torque_sensor_;
  const std::vector<std::vector<double>>* pid_gains_;
  const std::vector<double>* ff_term_;
};

/** \brief Hardware interface to support reading the state of an array of actuators
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * actuators, each of which has some position, velocity, and effort (force or
 * torque).
 *
 */
class ActuatorStateInterface : public HardwareResourceManager<ActuatorStateHandle> {};

}

#endif
