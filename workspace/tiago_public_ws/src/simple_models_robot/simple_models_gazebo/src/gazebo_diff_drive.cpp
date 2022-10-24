/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <algorithm>
#include <assert.h>

#include <simple_models_gazebo/gazebo_diff_drive.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <boost/preprocessor.hpp>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo
{
enum
{
  RIGHT,
  LEFT,
};

GazeboDiffDrive::GazeboDiffDrive()
{
}

// Destructor
GazeboDiffDrive::~GazeboDiffDrive()
{
}

void GazeboDiffDrive::LoadGeneric(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model_ = _parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "DiffDrive"));
  // Make sure the ROS node for Gazebo has already been initialized
  gazebo_ros_->isInitialized();

  gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic", "cmd_vel");
  gazebo_ros_->getParameter<std::string>(odometry_topic_, "odometryTopic", "odom");
  gazebo_ros_->getParameter<std::string>(odometry_frame_, "odometryFrame", "odom");
  gazebo_ros_->getParameter<std::string>(robot_base_frame_, "robotBaseFrame", "base_footprint");
  gazebo_ros_->getParameterBoolean(legacy_mode_, "legacyMode", true);

  if (!_sdf->HasElement("legacyMode"))
  {
    ROS_ERROR_NAMED(
        "diff_drive",
        "GazeboRosDiffDrive Plugin missing <legacyMode>, defaults to true\n"
        "This setting assumes you have a old package, where the right and left wheel are changed to fix a former code issue\n"
        "To get rid of this error just set <legacyMode> to false if you just created a new package.\n"
        "To fix an old package you have to exchange left wheel by the right wheel.\n"
        "If you do not want to fix this issue in an old package or your z axis points down instead of the ROS standard defined in REP 103\n"
        "just set <legacyMode> to true.\n");
  }

  gazebo_ros_->getParameter<double>(wheel_separation_, "wheelSeparation", 0.34);
  gazebo_ros_->getParameter<double>(wheel_diameter_, "wheelDiameter", 0.15);
  gazebo_ros_->getParameter<double>(wheel_accel, "wheelAcceleration", 0.0);
  gazebo_ros_->getParameter<double>(wheel_torque, "wheelTorque", 5.0);
  gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);

  std::map<std::string, OdomSource> odomOptions;
  odomOptions["encoder"] = ENCODER;
  odomOptions["world"] = WORLD;
  gazebo_ros_->getParameter<OdomSource>(odom_source_, "odometrySource", odomOptions, WORLD);

  this->publish_tf_ = true;
  if (!_sdf->HasElement("publishTf"))
  {
    ROS_WARN_NAMED("diff_drive",
                   "GazeboRosDiffDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
                   this->robot_namespace_.c_str(), this->publish_tf_);
  }
  else
  {
    this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
  }

  // Initialize update rate stuff
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0 / this->update_rate_;
  else
    this->update_period_ = 0.0;

#if BOOST_PP_LESS_EQUAL(IGNITION_MATH_MAJOR_VERSION, KINETIC_MATH_VERSION)
  last_update_time_ = model_->GetWorld()->GetSimTime();
  last_callback_time_ = model_->GetWorld()->GetSimTime();
#else
  last_update_time_ = model_->GetWorld()->SimTime();
  last_callback_time_ = model_->GetWorld()->SimTime();
#endif

  // Initialize velocity stuff
  wheel_speed_[RIGHT] = 0;
  wheel_speed_[LEFT] = 0;

  // Initialize velocity support stuff
  wheel_speed_instr_[RIGHT] = 0;
  wheel_speed_instr_[LEFT] = 0;

  x_ = 0;
  rot_ = 0;
  alive_ = true;

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ROS_INFO_NAMED("diff_drive", "%s: Try to subscribe to %s", gazebo_ros_->info(),
                 command_topic_.c_str());

  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      command_topic_, 1, boost::bind(&GazeboDiffDrive::cmdVelCallback, this, _1),
      ros::VoidPtr(), &queue_);

  cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
  ROS_INFO_NAMED("diff_drive", "%s: Subscribe to %s", gazebo_ros_->info(),
                 command_topic_.c_str());

  if (this->publish_tf_)
  {
    odometry_publisher_ =
        gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    ROS_INFO_NAMED("diff_drive", "%s: Advertise odom on %s ", gazebo_ros_->info(),
                   odometry_topic_.c_str());
  }

  transform_broadcaster_ =
      boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

  // start custom queue for diff drive
  this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboDiffDrive::QueueThread, this));
}

void GazeboDiffDrive::ResetGeneric()
{
#if BOOST_PP_LESS_EQUAL(IGNITION_MATH_MAJOR_VERSION, KINETIC_MATH_VERSION)
  last_update_time_ = model_->GetWorld()->GetSimTime();
#else
  last_update_time_ = model_->GetWorld()->SimTime();
#endif
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  rot_ = 0;
}

// Update the controller
void GazeboDiffDrive::UpdateChildGeneric()
{
  if (odom_source_ == ENCODER)
    UpdateOdometryEncoder();

#if BOOST_PP_LESS_EQUAL(IGNITION_MATH_MAJOR_VERSION, KINETIC_MATH_VERSION)
  current_time_ = model_->GetWorld()->GetSimTime();
#else
  current_time_ = model_->GetWorld()->SimTime();
#endif

  double seconds_since_last_callback_update = (current_time_ - last_callback_time_).Double();

  if (seconds_since_last_callback_update > update_period_)
  {
    x_ = 0.0;
    rot_ = 0.0;
  }

  seconds_since_last_update_ = (current_time_ - last_update_time_).Double();

  if (seconds_since_last_update_ > update_period_)
  {
    if (this->publish_tf_)
      publishOdometry(seconds_since_last_update_);

    last_update_time_ += common::Time(update_period_);
  }
}

// Finalize the controller
void GazeboDiffDrive::FiniChildGeneric()
{
  alive_ = false;
  queue_.clear();
  queue_.disable();
  gazebo_ros_->node()->shutdown();
  callback_queue_thread_.join();
}

void GazeboDiffDrive::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  x_ = cmd_msg->linear.x;
  rot_ = cmd_msg->angular.z;
#if BOOST_PP_LESS_EQUAL(IGNITION_MATH_MAJOR_VERSION, KINETIC_MATH_VERSION)
  last_callback_time_ = model_->GetWorld()->GetSimTime();
#else
  last_callback_time_ = model_->GetWorld()->SimTime();
#endif
}

void GazeboDiffDrive::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && gazebo_ros_->node()->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboDiffDrive::UpdateOdometryEncoder()
{
  boost::mutex::scoped_lock scoped_lock(lock);

  double wl = 0;
  double wr = 0;

  if (legacy_mode_)
  {
    wl = x_ + rot_ * wheel_separation_ / 2.0;
    wr = x_ - rot_ * wheel_separation_ / 2.0;
  }
  else
  {
    wl = x_ - rot_ * wheel_separation_ / 2.0;
    wr = x_ + rot_ * wheel_separation_ / 2.0;
  }

#if BOOST_PP_LESS_EQUAL(IGNITION_MATH_MAJOR_VERSION, KINETIC_MATH_VERSION)
  common::Time current_time = model_->GetWorld()->GetSimTime();
#else
  common::Time current_time = model_->GetWorld()->SimTime();
#endif

  double seconds_since_last_update = (current_time - last_odom_update_).Double();
  last_odom_update_ = current_time;

  double b = wheel_separation_;

  // Book: Sigwart 2011 Autonompus Mobile Robots page:337
  double sl = wl * (wheel_diameter_ / 2.0) * seconds_since_last_update;
  double sr = wr * (wheel_diameter_ / 2.0) * seconds_since_last_update;
  double ssum = sl + sr;

  double sdiff;
  if (legacy_mode_)
  {
    sdiff = sl - sr;
  }
  else
  {
    sdiff = sr - sl;
  }

  double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dtheta = (sdiff) / b;

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  double w = dtheta / seconds_since_last_update;
  double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

  tf::Quaternion qt;
  tf::Vector3 vt;
  qt.setRPY(0, 0, pose_encoder_.theta);
  vt = tf::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = dx / seconds_since_last_update;
  odom_.twist.twist.linear.y = dy / seconds_since_last_update;
}

void GazeboDiffDrive::publishOdometry(double step_time)
{
  ros::Time current_time = ros::Time::now();
  std::string odom_frame = gazebo_ros_->resolveTF(odometry_frame_);
  std::string base_footprint_frame = gazebo_ros_->resolveTF(robot_base_frame_);

  tf::Quaternion qt(0, 0, 0, 1);
  tf::Vector3 vt;

  if (odom_source_ == ENCODER)
  {
    // getting data form encoder integration
    qt = tf::Quaternion(odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,
                        odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
    vt = tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y,
                     odom_.pose.pose.position.z);
  }
  if (odom_source_ == WORLD)
  {
// getting data form gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = model_->WorldPose();
#else
    ignition::math::Pose3d pose = model_->GetWorldPose().Ign();
#endif
    qt = tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    vt = tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
#if BOOST_PP_LESS_EQUAL(IGNITION_MATH_MAJOR_VERSION, KINETIC_MATH_VERSION)
    linear = model_->GetWorldLinearVel().Ign();
    odom_.twist.twist.angular.z = model_->GetWorldAngularVel().Ign().Z();
#else
    linear = model_->WorldLinearVel();
    odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();
#endif

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
  }

  tf::Transform base_footprint_to_odom(qt, vt);
  transform_broadcaster_->sendTransform(tf::StampedTransform(
      base_footprint_to_odom, current_time, odom_frame, base_footprint_frame));


  // set covariance
  odom_.pose.covariance[0] = 0.00001;
  odom_.pose.covariance[7] = 0.00001;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = 0.001;


  // set header
  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;

  odometry_publisher_.publish(odom_);
}
}
