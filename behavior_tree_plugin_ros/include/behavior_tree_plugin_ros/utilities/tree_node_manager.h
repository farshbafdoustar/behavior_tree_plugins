#ifndef BEHAVIOR_TREE_PLUGIN_ROS_TREE_NODE_MANAGER_H
#define BEHAVIOR_TREE_PLUGIN_ROS_TREE_NODE_MANAGER_H

#include "behaviortree_cpp/action_node.h"
#include <ros/ros.h>
#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_types.h>

namespace behavior_tree_plugin_ros
{
class TreeNodeManager
{
public:
  TreeNodeManager(BT::TreeNode& tree_node);
  static void makePortList(BT::PortsList& local_port_list, const BT::PortDirection& port_direction,
                           const ros_babel_fish::MessageTemplate::ConstPtr message_template,
                           const std::string& prefix = "");
  bool fillMessageFromInputPorts(ros_babel_fish::Message& message, const std::string& prefix);
  void fillOutputPortsWithMessage(ros_babel_fish::Message& message, const std::string& prefix);

private:
  BT::TreeNode& tree_node_;
};

}  // namespace behavior_tree_plugin_ros
#endif  // BEHAVIOR_TREE_PLUGIN_ROS_TREE_NODE_MANAGER_H
