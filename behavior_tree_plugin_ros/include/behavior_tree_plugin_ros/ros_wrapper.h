#ifndef BEHAVIOR_TREE_PLUGIN_ROS_ROS_WRAPPER_H
#define BEHAVIOR_TREE_PLUGIN_ROS_ROS_WRAPPER_H

#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

namespace behavior_tree_plugin_ros
{
class RosWrapper : public BT::ControlNode
{
public:
  RosWrapper(const std::string& name, const BT::NodeConfig& config, ros::NodeHandle& node_handle);
  virtual ~RosWrapper() override = default;
  virtual void initialize();
  virtual BT::NodeStatus tick() override;
  void halt() override;
  static void Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                       ros::NodeHandle& node_handle);
  static BT::PortsList providedPorts();

  virtual std::string getStatusTopicName();
  virtual std::string getCommandTopicName();
  virtual int getIdleCode();
  virtual int getRunningCode();
  virtual int getSuccessCode();
  virtual int getFailureCode();
  void UpdateAndPublishChildrenStatus(int i, int status);
  virtual void OnChildSuccess(int i);
  virtual void OnChildFailure(int i);
  virtual void OnChildRunning(int i);
  virtual void OnChildCommandInitialize(int i);
  virtual void OnChildStatusInitialize(int i);
  virtual void OnHalt();

  virtual bool isRunAlwaysActive();

  virtual void onNewState(const std_msgs::BoolConstPtr& msg);

protected:
  ros::NodeHandle& node_handle_;
  std::vector<std_msgs::Int16> children_status_;
  std::vector<ros::Publisher> children_status_publisher_;

  std::vector<int> children_command_;
  std::vector<ros::Subscriber> children_command_subscriber_;
  std::vector<boost::function<void(const std_msgs::BoolConstPtr&, const int&, std::string&)>>
      children_command_call_back_;
  ros::Subscriber halt_subscriber_;
  ros::Time last_tick_time_;
  double frequency_{ 0.0 };

private:
};

}  // namespace behavior_tree_plugin_ros
#endif  // BEHAVIOR_TREE_PLUGIN_ROS_ROS_WRAPPER_H
