#include <joy_teleop/incrementer_server.h>


namespace pal
{
IncrementServer::IncrementServer(ros::NodeHandle &nh)
  : nh_(nh)
  , as_(nh_, "increment", boost::bind(&IncrementServer::executeCb, this, _1), false)
  , state_(nullptr)
{
  if (!urdf_model_.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
    return;
  }

  state_sub_ = nh_.subscribe("state", 1, &IncrementServer::StateCb, this);
  command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);

  ros::Time start_time = ros::Time::now();
  while (ros::ok() && (!state_) && ((ros::Time::now() - start_time) < ros::Duration(20.0)))
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  if (!state_)
  {
    ROS_ERROR_STREAM("Failed to receive message from " << nh_.getNamespace() << "/state");
    return;
  }

  joint_names_ = state_->joint_names;

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    double lower = urdf_model_.getJoint(joint_names_[i])->limits->lower;
    double upper = urdf_model_.getJoint(joint_names_[i])->limits->upper;
    joint_limits_.push_back(std::make_pair(lower, upper));
  }

  as_.start();
}

IncrementServer::~IncrementServer()
{
  as_.shutdown();
}

void IncrementServer::executeCb(const teleop_tools_msgs::IncrementGoalConstPtr &increment)
{
  if ((ros::Time::now() - state_->header.stamp) > ros::Duration(0.1))
  {
    ROS_ERROR_STREAM("Couldn't receive msg from " << nh_.getNamespace() << "/state");
    as_.setAborted();
  }

  if (increment->increment_by.size() != joint_names_.size())
  {
    ROS_ERROR_STREAM(nh_.getNamespace()
                     << "/increment size doesn't coincide with the number of controlled joints");
    as_.setAborted();
  }

  trajectory_msgs::JointTrajectory goal;
  goal.joint_names = joint_names_;
  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(0.1);
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    double position = state_->desired.positions[i] + increment->increment_by[i];
    if (position < joint_limits_[i].first)
    {
      ROS_WARN_STREAM(nh_.getNamespace()
                      << "/increment: Couldn't dicrease over the lower joint limit");
      position = joint_limits_[i].first;
    }
    else if (position > joint_limits_[i].second)
    {
      ROS_WARN_STREAM(nh_.getNamespace()
                      << "/increment: Couldn't increase over the upper joint limit");
      position = joint_limits_[i].second;
    }
    point.positions.push_back(position);
  }
  goal.points.push_back(point);
  goal.header.stamp = ros::Time::now();
  command_pub_.publish(goal);
  as_.setSucceeded();
}

void IncrementServer::StateCb(const control_msgs::JointTrajectoryControllerStateConstPtr &msg)
{
  state_ = msg;
}
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "incrementer_server");
  ros::NodeHandle nh("~");
  std::string controller_name = nh.getNamespace();

  if (!controller_name.empty())
  {
    std::size_t found = controller_name.find_last_of("/");
    if (found != std::string::npos)
      controller_name = controller_name.erase(found, controller_name.size() - 1);

    controller_name.erase(0);
  }
  ros::NodeHandle controller_nh(controller_name);
  pal::IncrementServer increment_server(controller_nh);

  ros::spin();

  return (0);
}
