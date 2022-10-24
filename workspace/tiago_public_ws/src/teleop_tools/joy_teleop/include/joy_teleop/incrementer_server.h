#include <trajectory_msgs/JointTrajectory.h>
#include <teleop_tools_msgs/IncrementAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <actionlib/server/simple_action_server.h>
#include <urdf/model.h>

namespace pal
{
typedef actionlib::SimpleActionServer<teleop_tools_msgs::IncrementAction> SActionServer;

class IncrementServer
{
public:
  IncrementServer(ros::NodeHandle &nh);

  ~IncrementServer();

private:
  void executeCb(const teleop_tools_msgs::IncrementGoalConstPtr &increment);

  void StateCb(const control_msgs::JointTrajectoryControllerStateConstPtr &msg);

  ros::NodeHandle nh_;
  SActionServer as_;
  urdf::Model urdf_model_;
  ros::Subscriber state_sub_;
  ros::Publisher command_pub_;
  control_msgs::JointTrajectoryControllerStateConstPtr state_;
  std::vector<std::string> joint_names_;
  std::vector<std::pair<double, double>> joint_limits_;
};
}
