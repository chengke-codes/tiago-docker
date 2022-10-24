/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef GAZEBO_FORCE_CONTROL_H
#define GAZEBO_FORCE_CONTROL_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

namespace gazebo {

  class Entity;

  class GazeboForceControl : public ModelPlugin {

  public:
    GazeboForceControl();
    ~GazeboForceControl();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  protected:
    virtual void GazeboUpdate();

  private:
    void RosQueueThread();
    void desiredPoseCb(const geometry_msgs::Pose::ConstPtr &msg);

    event::ConnectionPtr update_connection_;
    physics::ModelPtr model_;
    physics::LinkPtr link_;
    std::string robot_namespace_;
    std::string link_name_;
    double update_rate_;

    ros::NodeHandle nh_;
    ros::CallbackQueue callback_queue_;
    boost::thread callback_queue_thread_;
    boost::mutex mutex_;
    ros::Subscriber pose_sub_;

    geometry_msgs::Pose received_pose_;
  };

}

#endif // GAZEBO_FORCE_CONTROL_H
