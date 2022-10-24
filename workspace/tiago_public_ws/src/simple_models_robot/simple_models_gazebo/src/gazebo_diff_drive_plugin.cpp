/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <simple_models_gazebo/gazebo_diff_drive_plugin.h>

namespace gazebo
{
GazeboDiffDrivePlugin::GazeboDiffDrivePlugin()
{
}

GazeboDiffDrivePlugin::~GazeboDiffDrivePlugin()
{
}

// Load the controller
void GazeboDiffDrivePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  diff_drive_.LoadGeneric(_parent, _sdf);
  diff_drive_.gazebo_ros_->getParameter<bool>(gravity_, "gravity", true);
  diff_drive_.model_->SetGravityMode(gravity_);
  diff_drive_.gazebo_ros_->getParameter<double>(Kp_, "Kp", 1.0);
  diff_drive_.gazebo_ros_->getParameter<double>(Kw_, "Kw", 1.0);

//  prev_vt_ = tf::Vector3(0, 0, 0);
//  prev_qt_ = tf::Quaternion(0, 0, 0, 1);
//  diff_drive_.current_time_ = diff_drive_.model_->GetWorld()->GetSimTime();

  // listen to the update event (broadcast every simulation iteration)
  diff_drive_.update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboDiffDrivePlugin::UpdateChild, this));
}

void GazeboDiffDrivePlugin::Reset()
{
  diff_drive_.ResetGeneric();
}

void GazeboDiffDrivePlugin::UpdateChild()
{
  boost::mutex::scoped_lock scoped_lock(diff_drive_.lock);

//  common::Time prev_time = diff_drive_.current_time_;

  diff_drive_.UpdateChildGeneric();

  //  if (diff_drive_.seconds_since_last_update_ > diff_drive_.update_period_)
  {
#if BOOST_PP_LESS_EQUAL(IGNITION_MATH_MAJOR_VERSION, KINETIC_MATH_VERSION)
    ignition::math::Vector3d linear_vel = diff_drive_.model_->GetWorldPose().Ign().Rot() *
                                          ignition::math::Vector3d(diff_drive_.x_, 0, 0);
#else
    ignition::math::Vector3d linear_vel =
        diff_drive_.model_->WorldPose().Rot() * ignition::math::Vector3d(diff_drive_.x_, 0, 0);
#endif
    linear_vel.Z(0.0);

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = diff_drive_.model_->WorldPose();
#else
    ignition::math::Pose3d pose = diff_drive_.model_->GetWorldPose().Ign();
#endif
//    tf::Quaternion qt = tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
//    tf::Vector3 vt = tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

//    tf::Vector3 curr_lin_vel = (vt - prev_vt_)/(diff_drive_.current_time_ - prev_time).Double();
//    double curr_ang_vel = (tf::getYaw(qt) - tf::getYaw(prev_qt_))/(diff_drive_.current_time_ - prev_time).Double();
//    ignition::math::Vector3d curr_lin_vel_ign(curr_lin_vel.getX(), curr_lin_vel.getY(), curr_lin_vel.getZ());

//    ignition::math::Vector3d pos_error =  Kp_ * (linear_vel - curr_lin_vel_ign);
//    ignition::math::Vector3d qt_error =  Kp_ * (ignition::math::Vector3d(0, 0, diff_drive_.rot_) -
//                                                ignition::math::Vector3d(0, 0, curr_ang_vel));
//    pos_error.Z(0.0);

//    ROS_WARN_STREAM("LINEAR VEL " << pos_error);
//    ROS_WARN_STREAM("ANGULAR VEL " << qt_error);

    diff_drive_.model_->SetWorldTwist(
        Kp_ * linear_vel, Kw_ * ignition::math::Vector3d(0, 0, diff_drive_.rot_));

//    prev_qt_ = qt;
//    prev_vt_ = vt;
  }
}

void GazeboDiffDrivePlugin::FiniChild()
{
  diff_drive_.FiniChildGeneric();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboDiffDrivePlugin)
}
