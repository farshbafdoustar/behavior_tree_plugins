#ifndef BEHAVIOR_TREE_PLUGIN_ROS_ROS_WRAPPER_H
#define BEHAVIOR_TREE_PLUGIN_ROS_ROS_WRAPPER_H


#include <behaviortree_cpp_v3/control_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>


namespace behavior_tree_plugin_ros
{
class RosWrapper : public BT::ControlNode
{
public:

  RosWrapper(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle& node_handle);
  virtual ~RosWrapper() override = default;
  void initialize();
  BT::NodeStatus tick() override;
  void halt() override;
  static void Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                              ros::NodeHandle& node_handle);
  static BT::PortsList providedPorts();

std::string getStatusTopicName();
std::string getCommandTopicName();
int getIdleCode();
int getRunningCode();
int getSuccessCode();
int getFailureCode();

virtual bool isRunAlwaysActive();
private:
  ros::NodeHandle& node_handle_;
  std::vector<ros::Subscriber> children_run_command_subscriber_;
  std::vector<ros::Publisher> children_status_publisher_;

  std::vector<int> children_run_command_;
  std::vector<boost::function<void (const std_msgs::BoolConstPtr&,const int&,std::string &)>> children_run_command_call_back_;
  std::vector<std_msgs::Int32> children_status;


};

}  // namespace behavior_tree_plugin_ros
#endif  // BEHAVIOR_TREE_PLUGIN_ROS_ROS_WRAPPER_H