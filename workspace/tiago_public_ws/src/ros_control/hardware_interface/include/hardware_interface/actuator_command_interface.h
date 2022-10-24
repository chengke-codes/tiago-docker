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

#ifndef HARDWARE_INTERFACE_ACTUATOR_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_ACTUATOR_COMMAND_INTERFACE_H

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/actuator_state_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single actuator. */
class ActuatorHandle : public ActuatorStateHandle
{
public:
  ActuatorHandle()
    : ActuatorStateHandle(), cmd_(nullptr), pid_gains_cmd_(nullptr), ff_term_cmd_(nullptr)
  {
  }

  /**
   * \param as This actuator's state handle
   * \param cmd A pointer to the storage for this actuator's output command
   * \param pid_gains_cmd A pointer to the storage for this actuator's PIDs command
   * \param ff_term_cmd A pointer to the storage for this actuator's FF term command
   */
  ActuatorHandle(const ActuatorStateHandle& as, double* cmd,
                 std::vector<std::vector<double>>* pid_gains_cmd = nullptr,
                 std::vector<double>* ff_term_cmd = nullptr)
    : ActuatorStateHandle(as), cmd_(cmd), pid_gains_cmd_(pid_gains_cmd), ff_term_cmd_(ff_term_cmd)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + as.getName() +
                                       "'. Command data pointer is null.");
    }
    if(pid_gains_cmd_ && pid_gains_cmd_->size() != 3)
    {
      throw HardwareInterfaceException("Cannot create handle '" + as.getName() +
                                       "'. The parsed PID gains command pointer is not of size corresponding to modes size (3).");
    }
    if(pid_gains_cmd_)
    {
      for (const std::vector<double>& pids : *pid_gains_cmd_)
      {
        if (pids.size() != 3)
        {
          throw HardwareInterfaceException("Cannot create handle '" + as.getName() +
                                           "'. The parsed PID gains command is not of size 3.");
        }
      }
    }
    if (pid_gains_cmd_ && ff_term_cmd_ && pid_gains_cmd_->size() != ff_term_cmd_->size())
    {
      throw HardwareInterfaceException("Cannot create handle '" + as.getName() +
                                       "'. The parsed PID gains and FF Term are not of same size.");
    }
  }

  void setCommand(double command) {assert(cmd_); *cmd_ = command;}
  double getCommand() const {assert(cmd_); return *cmd_;}

  double* getCommandPtr() {return cmd_;}

  // Methods for setting and getting the gains information

  void setPIDGainsCmd(const std::vector<std::vector<double>> &modePIDs)
  {
    assert(pid_gains_cmd_);
    assert(pid_gains_cmd_->size() == modePIDs.size());
    for(size_t i = 0; i < modePIDs.size(); i++)
    {
      assert(modePIDs[i].size() == 3);
      (*pid_gains_cmd_)[i] = modePIDs[i];
    }
  }

  void setPIDGainsCmd(const std::vector<double> &modePIDs, unsigned int mode)
  {
    assert(pid_gains_cmd_);
    assert(mode < pid_gains_cmd_->size());
    for(size_t i = 0; i < modePIDs.size(); i++)
    {
      assert(modePIDs.size() == (*pid_gains_cmd_)[mode].size());
      assert(modePIDs.size() == 3);
      (*pid_gains_cmd_)[mode][i] = modePIDs[i];
    }
  }

  std::vector<std::vector<double>> getPIDGainsCmd() const
  {
    assert(pid_gains_cmd_);
    if (!pid_gains_cmd_)
    {
      throw std::runtime_error("Actuator : " + getName() + "does not support PID gains");
    }
    return *pid_gains_cmd_;
  }

  const std::vector<std::vector<double>>* getPIDGainsCmdConstPtr() const
  {
    return pid_gains_cmd_;
  }

  void setFFTermCmd(const std::vector<double>& ff_gain)
  {
    assert(ff_term_cmd_);
    *ff_term_cmd_ = ff_gain;
  }

  void setFFTermCmd(double ff_gain, unsigned int mode)
  {
    assert(ff_term_cmd_);
    assert(mode < ff_term_cmd_->size());
    (*ff_term_cmd_)[mode] = ff_gain;
  }

  std::vector<double> getFFTermCmd() const
  {
    assert(ff_term_cmd_);
    return *ff_term_cmd_;
  }

  const std::vector<double>* getFFTermCmdConstPtr() const
  {
    return ff_term_cmd_;
  }

private:
  double* cmd_;
  std::vector<std::vector<double>>* pid_gains_cmd_;
  std::vector<double>* ff_term_cmd_;
};

/** \brief Hardware interface to support commanding an array of actuators.
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named actuators. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortActuatorInterface etc.
 *
 */
class ActuatorCommandInterface : public HardwareResourceManager<ActuatorHandle> {};

/// \ref ActuatorCommandInterface for commanding effort-based actuators
class EffortActuatorInterface : public ActuatorCommandInterface {};

/// \ref ActuatorCommandInterface for commanding velocity-based actuators
class VelocityActuatorInterface : public ActuatorCommandInterface {};

/// \ref ActuatorCommandInterface for commanding position-based actuators
class PositionActuatorInterface : public ActuatorCommandInterface {};

}

#endif
