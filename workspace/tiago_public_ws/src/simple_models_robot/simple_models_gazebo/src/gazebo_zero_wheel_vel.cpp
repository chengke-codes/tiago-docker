/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <simple_models_gazebo/gazebo_zero_wheel_vel.h>

namespace gazebo
{

GazeboZeroWheelVel::GazeboZeroWheelVel()
{

}

GazeboZeroWheelVel::~GazeboZeroWheelVel()
{

}

void GazeboZeroWheelVel::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model_ = _parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "ZeroWheelVel"));
  // Make sure the ROS node for Gazebo has already been initialized
  gazebo_ros_->isInitialized();

  gazebo_ros_->getParameter<std::string>(wheel_name_, "wheelName", "wheel");

  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboZeroWheelVel::UpdateChild, this));
}

void GazeboZeroWheelVel::UpdateChild()
{
  this->model_->GetJoint(wheel_name_)->SetVelocity(0, 0.0);
  this->model_->GetJoint(wheel_name_)->SetPosition(0, 0.0);

//  this->model_->GetLink("wheel_back_left_link")->SetWorldTwist(math::Vector3(0,0,0), math::Vector3(0,0,0));
//  this->model_->GetLink("wheel_back_right_link")->SetWorldTwist(math::Vector3(0,0,0), math::Vector3(0,0,0));
//  this->model_->GetLink("wheel_front_left_link")->SetWorldTwist(math::Vector3(0,0,0), math::Vector3(0,0,0));
//  this->model_->GetLink("wheel_front_right_link")->SetWorldTwist(math::Vector3(0,0,0), math::Vector3(0,0,0));
//  this->model_->GetLink("wheel_back_left_link")->SetLinearVel(math::Vector3(0,0,0));
//  this->model_->GetLink("wheel_back_right_link")->SetLinearVel(math::Vector3(0,0,0));
//  this->model_->GetLink("wheel_front_left_link")->SetLinearVel(math::Vector3(0,0,0));
//  this->model_->GetLink("wheel_front_right_link")->SetLinearVel(math::Vector3(0,0,0));
//  const auto &jointController = this->model_->GetJointController();
//  jointController->SetVelocityTarget(wheel_name_, 0.0);
//  jointController->SetPositionTarget(wheel_name_, 0.0);
}

void GazeboZeroWheelVel::FiniChild()
{
  gazebo_ros_->node()->shutdown();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboZeroWheelVel)

}
