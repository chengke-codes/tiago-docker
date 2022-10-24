/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

namespace pal
{
class OdomTf
{
public:
  OdomTf();
  void getInfo(const nav_msgs::Odometry::ConstPtr& msg);
  ros::Time current_time;
  void run();
  nav_msgs::Odometry odom;
  tf2_ros::TransformBroadcaster odom_broadcaster;
  int pub_freq;
  std::string odom_frame, robot_frame, prefix;

private:
  ros::NodeHandle nh_;
  ros::Subscriber data_sub_;
  ros::Publisher data_pub_;
};

OdomTf::OdomTf()
{
  ros::NodeHandle priv_nh("");
  std::string robot_odom_topic("odom");
  data_sub_ = nh_.subscribe("ground_truth_odom", 1, &OdomTf::getInfo, this,
                            ros::TransportHints().tcpNoDelay(true));
  data_pub_ = nh_.advertise<nav_msgs::Odometry>(robot_odom_topic, 10);
  priv_nh.param<int>("pub_freq", pub_freq, 10);
  priv_nh.param<std::string>("robot_frame", robot_frame, "base_footprint");
  priv_nh.param<std::string>("odom_frame", odom_frame, "odom");
  priv_nh.param<std::string>("prefix", prefix, "");
  if (!prefix.empty())
  {
    odom_frame = prefix + "/" + odom_frame;
    robot_frame = prefix + "/" + robot_frame;
  }
}
void OdomTf::getInfo(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::TransformStamped odom_trans;

  current_time = ros::Time::now();

  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = odom_frame;
  odom_trans.child_frame_id = robot_frame;
  odom_trans.transform.translation.x = msg->pose.pose.position.x;
  odom_trans.transform.translation.y = msg->pose.pose.position.y;
  odom_trans.transform.translation.z = msg->pose.pose.position.z;

  odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
  odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
  odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
  odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;

  odom_broadcaster.sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = robot_frame;

  // set the position
  odom.pose.pose.position.x = msg->pose.pose.position.x;
  odom.pose.pose.position.y = msg->pose.pose.position.y;
  odom.pose.pose.position.z = msg->pose.pose.position.z;
  odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;

  odom.twist.twist.linear.x = msg->twist.twist.linear.x;
  odom.twist.twist.linear.y = msg->twist.twist.linear.y;
  odom.twist.twist.angular.z = msg->twist.twist.linear.z;
}

void OdomTf::run()
{
  ros::Rate odom_frequency(pub_freq);
  while (ros::ok())
  {
    ros::spinOnce();
    data_pub_.publish(odom);
    odom_frequency.sleep();
  }
}
} // namespace pal

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_odom_publisher");
  pal::OdomTf odomtf;
  odomtf.run();
  return 0;
}
