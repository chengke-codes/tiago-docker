/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef GAZEBO_ZERO_WHEEL_VEL_H
#define GAZEBO_ZERO_WHEEL_VEL_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ros/ros.h>

namespace gazebo
{

class GazeboZeroWheelVel : public ModelPlugin
{
public:
    GazeboZeroWheelVel();
    ~GazeboZeroWheelVel();
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

protected:
  virtual void UpdateChild();
  virtual void FiniChild();

private:
    std::string wheel_name_;

    GazeboRosPtr gazebo_ros_;
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
};
}

#endif // GAZEBO_ZERO_WHEEL_VEL_H
