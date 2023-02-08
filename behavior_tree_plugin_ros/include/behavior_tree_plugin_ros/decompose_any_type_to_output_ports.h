#ifndef BEHAVIOR_TREE_PLUGIN_ROS_DECOMPOSE_ANY_TYPE_TO_OUTPUT_PORTS_H
#define BEHAVIOR_TREE_PLUGIN_ROS_DECOMPOSE_ANY_TYPE_TO_OUTPUT_PORTS_H

#include "behaviortree_cpp/action_node.h"
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_types.h>
#include <ros_babel_fish/babel_fish_message.h>

#include "behavior_tree_plugin_ros/utilities/tree_node_manager.h"

namespace behavior_tree_plugin_ros
{
class DecomposeAnyTypeToOutputPorts : public BT::SyncActionNode
{
public:
  DecomposeAnyTypeToOutputPorts(const std::string& name, const BT::NodeConfig& config, ros::NodeHandle& node_handle,
                                std::string instance_type);
  static BT::PortsList getPorts(std::string instance_type);
  BT::NodeStatus tick() override;
  static void Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                       ros::NodeHandle& node_handle, std::string instance_type);

private:
  ros::NodeHandle& node_handle_;
  const std::string instance_type_;

  ros_babel_fish::BabelFish* fish_;
  TreeNodeManager* tree_node_manager_;
};

}  // namespace behavior_tree_plugin_ros
#endif  // BEHAVIOR_TREE_PLUGIN_ROS_DECOMPOSE_ANY_TYPE_TO_OUTPUT_PORTS_H
