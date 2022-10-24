/*
  @file gtest_robot_pose.cpp

  @author Procópio Stein

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Odometry.h>

namespace pal
{
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  ROS_INFO_STREAM(msg);
  EXPECT_TRUE(msg != NULL);
  EXPECT_NEAR(0.0, msg->pose.pose.position.x, 0.001);
  EXPECT_NEAR(0.0, msg->pose.pose.position.y, 0.001);
  EXPECT_NEAR(0.0, msg->pose.pose.position.z, 0.001);
  EXPECT_NEAR(0.0, msg->pose.pose.orientation.x, 0.001);
  EXPECT_NEAR(0.0, msg->pose.pose.orientation.y, 0.001);
  EXPECT_NEAR(0.0, msg->pose.pose.orientation.z, 0.001);
  EXPECT_NEAR(1.0, msg->pose.pose.orientation.w, 0.1);
}

TEST(FakeOdom, test)
{
  ros::NodeHandle nh;
  ROS_INFO("wait for /odom");
  ros::Subscriber sub = nh.subscribe("/odom", 1000, odomCallback);
  ros::Duration(2.0).sleep();
}

TEST(FakeTf, test)
{
  ros::NodeHandle nh;
  tf2_ros::Buffer tf_buffer;
  geometry_msgs::TransformStamped origin;
  ros::Duration(2.0).sleep();
  try
  {
    origin = tf_buffer.lookupTransform("odom", "base_footprint", ros::Time::now());
    ASSERT_EQ("base_footprint", origin.header.frame_id);
    EXPECT_NEAR(0.0, origin.transform.translation.x, 0.01);
    EXPECT_NEAR(0.0, origin.transform.translation.y, 0.01);
    EXPECT_NEAR(0.0, origin.transform.translation.z, 0.01);
    EXPECT_NEAR(0.0, origin.transform.rotation.x, 0.01);
    EXPECT_NEAR(0.0, origin.transform.rotation.y, 0.01);
    EXPECT_NEAR(0.0, origin.transform.rotation.z, 0.01);
    EXPECT_NEAR(1.0, origin.transform.rotation.w, 0.01);
  }
  catch (tf2::TransformException &ex)
  {
    ASSERT_NO_THROW(tf_buffer.canTransform("odom", "base_footprint", ros::Time::now()));
  }
}
}  // namespace pal

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_odom_test");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
