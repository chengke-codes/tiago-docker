/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef GAZEBO_DIFF_DRIVE_H
#define GAZEBO_DIFF_DRIVE_H

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#define KINETIC_MATH_VERSION 2
#define MELODIC_MATH_VERSION 4

namespace gazebo
{
class Joint;
class Entity;

class GazeboDiffDrive
{
  enum OdomSource
  {
    ENCODER = 0,
    WORLD = 1,
  };

public:
  GazeboDiffDrive();
  ~GazeboDiffDrive();
  void ResetGeneric();
  void UpdateChildGeneric();
  void FiniChildGeneric();

  void publishOdometry(double step_time);
  void UpdateOdometryEncoder();

  void LoadGeneric(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  GazeboRosPtr gazebo_ros_;
  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  double wheel_separation_;
  double wheel_diameter_;
  double wheel_torque;
  double wheel_speed_[2];
  double wheel_accel;
  double wheel_speed_instr_[2];

  // ROS STUFF
  ros::Publisher odometry_publisher_;
  ros::Subscriber cmd_vel_subscriber_;
  nav_msgs::Odometry odom_;

  boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;

  boost::mutex lock;

  std::string robot_namespace_;
  std::string command_topic_;
  std::string odometry_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;
  bool publish_tf_;
  bool legacy_mode_;
  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  // DiffDrive stuff
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  double x_;
  double rot_;
  bool alive_;

  // Update Rate
  double update_rate_;
  double update_period_;
  common::Time last_update_time_;
  common::Time last_callback_time_;

  OdomSource odom_source_;
  geometry_msgs::Pose2D pose_encoder_;
  common::Time last_odom_update_;
  common::Time current_time_;
  double seconds_since_last_update_;
};
}

#endif  // GAZEBO_DIFF_DRIVE_H
