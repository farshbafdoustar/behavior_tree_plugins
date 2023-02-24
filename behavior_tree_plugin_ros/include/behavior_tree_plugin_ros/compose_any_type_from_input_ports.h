#ifndef BEHAVIOR_TREE_PLUGIN_ROS_COMPOSE_ANY_TYPE_FROM_INPUT_PORTS_H
#define BEHAVIOR_TREE_PLUGIN_ROS_COMPOSE_ANY_TYPE_FROM_INPUT_PORTS_H

#include "behaviortree_cpp/action_node.h"
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_types.h>
#include <ros_babel_fish/babel_fish_message.h>

#include "behavior_tree_plugin_ros/utilities/tree_node_manager.h"

namespace behavior_tree_plugin_ros
{
class ComposeAnyTypeFromInputPorts : public BT::SyncActionNode
{
public:
  ComposeAnyTypeFromInputPorts(const std::string& name, const BT::NodeConfig& config, ros::NodeHandle& node_handle,
                               std::string instance_type);
  ComposeAnyTypeFromInputPorts(const std::string& name, const BT::NodeConfig& config, ros::NodeHandle& node_handle,
                               std::string instance_type, ros_babel_fish::BabelFish* fish_ptr);
  static BT::PortsList getPorts(std::string instance_type);
  static BT::PortsList getPorts(std::string instance_type, ros_babel_fish::BabelFish& fish);
  BT::NodeStatus tick() override;
  static void Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                       ros::NodeHandle& node_handle, std::string instance_type);
  static void Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                       ros::NodeHandle& node_handle, std::string instance_type, ros_babel_fish::BabelFish* fish);

private:
  ros::NodeHandle& node_handle_;
  const std::string instance_type_;

  ros_babel_fish::BabelFish* fish_;
  TreeNodeManager* tree_node_manager_;
  ros_babel_fish::Message::Ptr instance_ptr_;  // Message is Abstract type so only pointers are possible here
};

}  // namespace behavior_tree_plugin_ros
#endif  // BEHAVIOR_TREE_PLUGIN_ROS_COMPOSE_ANY_TYPE_FROM_INPUT_PORTS_H
