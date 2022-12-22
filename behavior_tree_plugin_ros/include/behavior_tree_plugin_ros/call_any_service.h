#ifndef BEHAVIOR_TREE_PLUGIN_ROS_CALL_ANY_SERVICE_H
#define BEHAVIOR_TREE_PLUGIN_ROS_CALL_ANY_SERVICE_H

#include "behaviortree_cpp/action_node.h"
#include <behaviortree_cpp/bt_factory.h>

#include <ros/ros.h>

namespace behavior_tree_plugin_ros
{
class CallAnyService : public BT::SyncActionNode
{
public:
  CallAnyService(const std::string& name, const BT::NodeConfig& config, std::string service_type);
  static BT::PortsList getPorts(std::string service_type);
  BT::NodeStatus tick() override;
  static void Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID, std::string service_type);

private:
  const std::string service_type_;
};

}  // namespace behavior_tree_plugin_ros
#endif  // BEHAVIOR_TREE_PLUGIN_ROS_CALL_ANY_SERVICE_H
