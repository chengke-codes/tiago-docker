// Copyright 2021 PAL Robotics SL.
// All Rights Reserved
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <hardware_interface/actuator_state_interface.h>
#include <actuator_state_controller/actuator_state_controller.h>

using namespace actuator_state_controller;

class ActuatorStateControllerTest : public ::testing::Test
{
public:
  ActuatorStateControllerTest()
    : root_nh_(ros::NodeHandle()), controller_nh_("test_ok/actuator_state_controller")
  {
    // Intialize raw actuator state data
    names_.push_back("actuator1");
    names_.push_back("actuator2");
    pos_[0] = 0.0;
    pos_[1] = 0.0;
    abs_pos_[0] = 0.0;
    abs_pos_[1] = 0.0;
    vel_[0] = 0.0;
    vel_[1] = 0.0;
    eff_[0] = 0.0;
    eff_[1] = 0.0;
    torque_[0] = 0.0;
    torque_[1] = 0.0;

    // Setup the actuator state interface
    hardware_interface::ActuatorStateHandle state_handle_1(names_[0], &pos_[0], &vel_[0],
                                                           &eff_[0]);
    js_iface_.registerHandle(state_handle_1);

    hardware_interface::ActuatorStateHandle state_handle_2(names_[1], &pos_[1], &vel_[1],
                                                           &eff_[1]);
    js_iface_.registerHandle(state_handle_2);


    // Setup the actuator state interface
    hardware_interface::ActuatorStateHandle state_handle_torque_1(
        names_[0], &pos_[0], &vel_[0], &eff_[0], &abs_pos_[0], &torque_[0]);
    js_iface_torque_.registerHandle(state_handle_torque_1);

    hardware_interface::ActuatorStateHandle state_handle_torque_2(
        names_[1], &pos_[1], &vel_[1], &eff_[1], &abs_pos_[1], &torque_[1]);
    js_iface_torque_.registerHandle(state_handle_torque_2);

    // Initialize ROS interfaces
    sub_ = root_nh_.subscribe<sensor_msgs::JointState>(
        "actuator_states", 1, &ActuatorStateControllerTest::actuatorStateCb, this);
  }

protected:
  ros::NodeHandle root_nh_;
  ros::NodeHandle controller_nh_;
  ros::Subscriber sub_;
  hardware_interface::ActuatorStateInterface js_iface_;
  hardware_interface::ActuatorStateInterface js_iface_torque_;

  // Raw actuator state data
  std::vector<std::string> names_;
  double pos_[2];
  double abs_pos_[2];
  double vel_[2];
  double eff_[2];
  double torque_[2];

  // Received actuator state messages counter
  int rec_msgs_;

  // Last received actuator state message
  sensor_msgs::JointState last_msg_;

  void actuatorStateCb(const sensor_msgs::JointStateConstPtr& msg)
  {
    last_msg_ = *msg;
    ++rec_msgs_;
  }
};

TEST_F(ActuatorStateControllerTest, initOk)
{
  ActuatorStateController jsc;
  EXPECT_TRUE(jsc.init(&js_iface_, root_nh_, controller_nh_));
}

TEST_F(ActuatorStateControllerTest, initKo)
{
  ActuatorStateController jsc;
  ros::NodeHandle bad_controller_nh("no_period_namespace");
  EXPECT_FALSE(jsc.init(&js_iface_, root_nh_, bad_controller_nh));
}

TEST_F(ActuatorStateControllerTest, publishOk)
{
  for (auto iface : { js_iface_, js_iface_torque_ })
  {
    ActuatorStateController jsc;
    EXPECT_TRUE(jsc.init(&iface, root_nh_, controller_nh_));

    int pub_rate;
    ASSERT_TRUE(controller_nh_.getParam("publish_rate", pub_rate));

    rec_msgs_ = 0;
    const int test_duration = 1;         // seconds
    const int loop_freq = 2 * pub_rate;  // faster than controller publish rate
    ros::Rate loop_rate(static_cast<double>(loop_freq));
    const ros::Time start_time = ros::Time::now();
    jsc.starting(start_time);

    while (true)
    {
      ros::Time now = ros::Time::now();
      jsc.update(now,
                 ros::Duration());  // NOTE: Second parameter is unused by implementation
      ros::spinOnce();
      if (rec_msgs_ >= test_duration * pub_rate)
        break;  // Objective reached
      if (now - start_time > ros::Duration(2.0 * test_duration))
        break;  // Prevents unexpected infinite loops
      loop_rate.sleep();
    }
    jsc.stopping(ros::Time::now());

    // NOTE: Below we subtract the loop rate because the the published message gets picked
    // up in the
    // next iteration's spinOnce()
    const ros::Duration real_test_duration =
        ros::Time::now() - start_time - loop_rate.expectedCycleTime();
    const double real_pub_rate = static_cast<double>(rec_msgs_) / real_test_duration.toSec();

    // The publish rate should be close to the nominal value
    EXPECT_NEAR(real_pub_rate, pub_rate, 0.05 * pub_rate);
  }
}

TEST_F(ActuatorStateControllerTest, publishKo)
{
  ActuatorStateController jsc;
  ros::NodeHandle negative_rate_nh("test_ko/actuator_state_controller");
  EXPECT_TRUE(jsc.init(&js_iface_, root_nh_, negative_rate_nh));

  // Check non-positive publish rate
  int pub_rate;
  ASSERT_TRUE(negative_rate_nh.getParam("publish_rate", pub_rate));
  ASSERT_LE(pub_rate, 0);

  rec_msgs_ = 0;
  const ros::Duration test_duration(1.0);
  ros::Rate loop_rate(10.0);
  const ros::Time start_time = ros::Time::now();
  while (ros::Time::now() - start_time < test_duration)
  {
    ros::Time now = ros::Time::now();
    jsc.update(now,
               ros::Duration());  // NOTE: Second parameter is unused by implementation
    ros::spinOnce();
    loop_rate.sleep();
  }
  jsc.stopping(ros::Time::now());

  // No messages should have been published
  EXPECT_EQ(rec_msgs_, 0);
}

TEST_F(ActuatorStateControllerTest, valuesOk)
{
  for (auto iface : { js_iface_, js_iface_torque_ })
  {
    ActuatorStateController jsc;
    EXPECT_TRUE(jsc.init(&iface, root_nh_, controller_nh_));

    int pub_rate;
    ASSERT_TRUE(controller_nh_.getParam("publish_rate", pub_rate));
    ros::Duration period(1.0 / pub_rate);
    jsc.starting(ros::Time::now());

    pos_[0] = 1.0;
    pos_[1] = -1.0;
    vel_[0] = 2.0;
    vel_[1] = -2.0;
    eff_[0] = 3.0;
    eff_[1] = -3.0;
    torque_[0] = 300.0;
    torque_[1] = -300.0;
    abs_pos_[0] = 10.0;
    abs_pos_[1] = -10.0;

    period.sleep();
    jsc.update(ros::Time::now(), ros::Duration());
    period.sleep();
    ros::spinOnce();  // To trigger callback

    // Check payload sizes
    ASSERT_EQ(names_.size(), last_msg_.name.size());
    ASSERT_EQ(last_msg_.name.size(), last_msg_.position.size());
    ASSERT_EQ(last_msg_.name.size(), last_msg_.velocity.size());
    ASSERT_EQ(last_msg_.name.size(), last_msg_.effort.size());

    // Check payload values
    typedef std::vector<std::string>::size_type SizeType;
    typedef std::vector<std::string>::iterator Iterator;
    for (SizeType raw_id = 0; raw_id < names_.size(); ++raw_id)
    {
      Iterator it = std::find(last_msg_.name.begin(), last_msg_.name.end(), names_[raw_id]);
      ASSERT_NE(last_msg_.name.end(), it);
      SizeType msg_id = std::distance(last_msg_.name.begin(), it);
      if (iface.getHandle(names_[raw_id]).hasAbsolutePosition())
      {
        EXPECT_EQ(abs_pos_[raw_id], last_msg_.position[msg_id]);
      }
      else
      {
        EXPECT_EQ(pos_[raw_id], last_msg_.position[msg_id]);
      }
      EXPECT_EQ(vel_[raw_id], last_msg_.velocity[msg_id]);
      if (iface.getHandle(names_[raw_id]).hasTorqueSensor())
      {
        EXPECT_EQ(torque_[raw_id], last_msg_.effort[msg_id]);
      }
      else
      {
        EXPECT_EQ(eff_[raw_id], last_msg_.effort[msg_id]);
      }
    }
    jsc.stopping(ros::Time::now());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "actuator_state_controller_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
