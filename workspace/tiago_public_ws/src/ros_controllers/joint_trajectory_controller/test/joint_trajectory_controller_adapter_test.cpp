/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
/** \author Sai Kishor Kothakota **/
#include <gtest/gtest.h>
#include <ros/node_handle.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>

TEST(JointTrajectoryControllerTest, CustomAdapterTest)
{
  joint_trajectory_controller::JointTrajectoryController<
      trajectory_interface::QuinticSplineSegment<double>,
      hardware_interface::EffortJointInterface, hardware_interface::EffortJointInterface>
      effort_trajectory_adapter;

  joint_trajectory_controller::JointTrajectoryController<
      trajectory_interface::QuinticSplineSegment<double>, hardware_interface::EffortJointInterface>
      effort_trajectory_no_adapter;

  // Not recommended in real scenario, but used here for testing purpose
  joint_trajectory_controller::JointTrajectoryController<
      trajectory_interface::QuinticSplineSegment<double>,
      hardware_interface::EffortJointInterface, hardware_interface::PositionJointInterface>
      effort_trajectory_position_adapter;

  joint_trajectory_controller::JointTrajectoryController<
      trajectory_interface::QuinticSplineSegment<double>,
      hardware_interface::PositionJointInterface, hardware_interface::EffortJointInterface>
      position_trajectory_effort_adapter;

  ros::NodeHandle nh, pnh;
  hardware_interface::EffortJointInterface effort_int;
  hardware_interface::PositionJointInterface pos_int;
  ASSERT_NO_THROW(effort_trajectory_adapter.init(&effort_int, nh, pnh));
  ASSERT_NO_THROW(effort_trajectory_no_adapter.init(&effort_int, nh, pnh));
  ASSERT_NO_THROW(effort_trajectory_position_adapter.init(&effort_int, nh, pnh));
  ASSERT_NO_THROW(position_trajectory_effort_adapter.init(&pos_int, nh, pnh));
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "joint_trajectory_controller_test");
  return RUN_ALL_TESTS();
}
