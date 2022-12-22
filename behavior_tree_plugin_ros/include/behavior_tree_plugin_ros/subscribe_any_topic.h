#ifndef BEHAVIOR_TREE_PLUGIN_ROS_SUBSCRIBE_ANY_TOPIC_H
#define BEHAVIOR_TREE_PLUGIN_ROS_SUBSCRIBE_ANY_TOPIC_H

#include "behaviortree_cpp/action_node.h"
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_types.h>
#include <ros_babel_fish/babel_fish_message.h>

namespace behavior_tree_plugin_ros
{
class SubscribeAnyTopic : public BT::SyncActionNode
{
public:
  SubscribeAnyTopic(const std::string& name, const BT::NodeConfig& config, ros::NodeHandle& node_handle,
                    std::string topic_type);
  static BT::PortsList getPorts(std::string service_type);
  BT::NodeStatus tick() override;
  static void Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                       ros::NodeHandle& node_handle, std::string topic_type);

private:
  std::string topic_name_;
  ros::NodeHandle& node_handle_;
  const std::string topic_type_;
  ros::Subscriber subscriber_;
  ros_babel_fish::BabelFishMessage::ConstPtr message_;
  void callback(const ros_babel_fish::BabelFishMessage::ConstPtr& message);
};

}  // namespace behavior_tree_plugin_ros
#endif  // BEHAVIOR_TREE_PLUGIN_ROS_SUBSCRIBE_ANY_TOPIC_H