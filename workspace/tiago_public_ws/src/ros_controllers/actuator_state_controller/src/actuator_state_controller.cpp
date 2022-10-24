// Copyright 2021 PAL Robotics SL.
// All Rights Reserved
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

/// \author Sai Kishor Kothakota
#include <algorithm>
#include <cstddef>
#include <actuator_state_controller/actuator_state_controller.h>

namespace actuator_state_controller
{
bool ActuatorStateController::init(hardware_interface::ActuatorStateInterface* hw,
                                   ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // get all actuator names from the hardware interface
  const std::vector<std::string>& actuator_names = hw->getNames();
  num_hw_actuators_ = actuator_names.size();
  for (unsigned i = 0; i < num_hw_actuators_; i++)
    ROS_DEBUG("Got actuator %s", actuator_names[i].c_str());

  // get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  // Get the name of the topic, if doesn't exist, use the actuator_states as default one
  std::string topic_name = controller_nh.param<std::string>("topic_name", "actuator_states");

  // Choose to publish absolute encoder position or not
  publish_absolute_position_ = controller_nh.param<bool>("publish_absolute_position", false);

  // Choose to publish the actuator current or the torque/force sensor readings
  publish_torque_ = controller_nh.param<bool>("publish_torque", true);

  // realtime publisher
  realtime_pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::JointState>>(
      root_nh, topic_name, 4);

  // get actuators and allocate message
  for (unsigned i = 0; i < num_hw_actuators_; i++)
  {
    actuator_state_.push_back(hw->getHandle(actuator_names[i]));
    realtime_pub_->msg_.name.push_back(actuator_names[i]);
    realtime_pub_->msg_.position.push_back(0.0);
    realtime_pub_->msg_.velocity.push_back(0.0);
    realtime_pub_->msg_.effort.push_back(0.0);
  }
  print_warnings_ = true;
  return true;
}

void ActuatorStateController::starting(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;
}

void ActuatorStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    // try to publish
    if (realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

      realtime_pub_->msg_.header.stamp = time;
      for (unsigned i = 0; i < num_hw_actuators_; i++)
      {
        if (publish_absolute_position_ && actuator_state_[i].hasAbsolutePosition())
        {
          realtime_pub_->msg_.position[i] = actuator_state_[i].getAbsolutePosition();
        }
        else
        {
          if (publish_absolute_position_ && print_warnings_)
          {
            ROS_WARN_STREAM("There is no absolute encoder on the actuator : "
                            << actuator_state_[i].getName()
                            << " , publishing incremental position instead!");
          }
          realtime_pub_->msg_.position[i] = actuator_state_[i].getPosition();
        }
        realtime_pub_->msg_.velocity[i] = actuator_state_[i].getVelocity();
        if (publish_torque_ && actuator_state_[i].hasTorqueSensor())
          realtime_pub_->msg_.effort[i] = actuator_state_[i].getTorqueSensor();
        else
        {
          if (publish_torque_ && print_warnings_)
            ROS_WARN_STREAM("There is no torque sensor on the actuator : "
                            << actuator_state_[i].getName() << " , publishing current instead!");
          realtime_pub_->msg_.effort[i] = actuator_state_[i].getEffort();
        }
      }
    }
    realtime_pub_->unlockAndPublish();
    print_warnings_ = false;
  }
}

void ActuatorStateController::stopping(const ros::Time& /*time*/)
{
}
}

PLUGINLIB_EXPORT_CLASS(actuator_state_controller::ActuatorStateController,
                       controller_interface::ControllerBase)
