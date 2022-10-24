/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <simple_models_gazebo/gazebo_force_control.h>

namespace gazebo
{
GazeboForceControl::GazeboForceControl() : ModelPlugin(), robot_namespace_("")
{
}

GazeboForceControl::~GazeboForceControl()
{
}

void GazeboForceControl::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("GazeboForceControl couldn't be loaded since ROS is not yet initialized");
    return;
  }

  if (_sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  if (_sdf->HasElement("linkName"))
  {
    link_name_ = _sdf->Get<std::string>("linkName");
  }
  else
  {
    ROS_FATAL("GazeboForceControl plugin requires a linkName parameter tag");
    return;
  }

  update_rate_ = 100.0;
  if (!_sdf->HasElement("updateRate"))
  {
    ROS_INFO_STREAM("GazeboForceControl plugin missing <updateRate>, defaults to " << update_rate_);
  }
  else
    update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

  model_ = _parent;
  link_ = model_->GetLink(link_name_);
  link_->SetGravityMode(false);
  nh_.setCallbackQueue(&callback_queue_);

  pose_sub_ = nh_.subscribe<geometry_msgs::Pose>("/robot_world_pose", 1,
                                                 &GazeboForceControl::desiredPoseCb, this);
  callback_queue_thread_ = boost::thread(boost::bind(&GazeboForceControl::RosQueueThread, this));

  update_connection_ =
      event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboForceControl::GazeboUpdate, this));
}

void GazeboForceControl::desiredPoseCb(const geometry_msgs::Pose::ConstPtr &msg)
{
  received_pose_ = *msg.get();
}

void GazeboForceControl::RosQueueThread()
{
  ros::Rate rate(update_rate_);

  while (nh_.ok())
  {
    callback_queue_.callAvailable();
    rate.sleep();
  }
}

void GazeboForceControl::GazeboUpdate()
{
  boost::mutex::scoped_lock sclock(mutex_);

  ignition::math::Pose3d desired(
      ignition::math::Vector3d(received_pose_.position.x, received_pose_.position.y,
                               received_pose_.position.z),
      ignition::math::Quaterniond(received_pose_.orientation.w, received_pose_.orientation.x,
                                  received_pose_.orientation.y, received_pose_.orientation.z));

#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d world_pose = link_->DirtyPose();
  ignition::math::Vector3d worldLinearVel = link_->WorldLinearVel();
#else
  ignition::math::Pose3d world_pose = link_->GetDirtyPose().Ign();
  ignition::math::Vector3d worldLinearVel = link_->GetWorldLinearVel().Ign();
#endif

  ignition::math::Vector3d err_pos = desired.Pos() - world_pose.Pos();

  link_->AddForce(1.0 * err_pos - worldLinearVel);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboForceControl)
}
