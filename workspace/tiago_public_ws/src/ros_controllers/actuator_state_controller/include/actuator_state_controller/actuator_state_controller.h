// Copyright 2021 PAL Robotics SL.
// All Rights Reserved
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.
#ifndef ACTUATOR_STATE_CONTROLLER_H
#define ACTUATOR_STATE_CONTROLLER_H
/// \author Sai Kishor Kothakota

#include <controller_interface/controller.h>
#include <hardware_interface/actuator_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

namespace actuator_state_controller
{
/**
 * \brief Controller that publishes the state of all actuators in a robot.
 *
 * This controller publishes the state of all resources registered to a \c
 * hardware_interface::ActuatorStateInterface to a  topic of type \c
 * sensor_msgs/JointState. The following is a basic configuration of
 * the controller.
 *
 * \code
 * actuator_state_controller:
 *   type: actuator_state_controller/ActuatorStateController
 *   publish_rate: 50
 *   topic_name: "actuator_states" # if not set, uses actuator_states as default
 *   publish_absolute_position: false # if not set, uses incremental encoder values
 *   publish_torque: false # If set to true, uses the torque or force sensor attached to
 *                    the actuator, if not set, publishes current by default
 * \endcode
 *
 * An unspecified position, velocity or acceleration defaults to zero.
 */
class ActuatorStateController
    : public controller_interface::Controller<hardware_interface::ActuatorStateInterface>
{
public:
  ActuatorStateController() : publish_rate_(0.0)
  {
  }

  virtual bool init(hardware_interface::ActuatorStateInterface* hw,
                    ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  std::vector<hardware_interface::ActuatorStateHandle> actuator_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;
  ros::Time last_publish_time_;
  double publish_rate_;
  bool print_warnings_;
  bool publish_absolute_position_, publish_torque_;
  unsigned int num_hw_actuators_;  ///< Number of actuators present in the
                                   /// ActuatorStateInterface,
};
}
#endif  // ACTUATOR_STATE_CONTROLLER_H
