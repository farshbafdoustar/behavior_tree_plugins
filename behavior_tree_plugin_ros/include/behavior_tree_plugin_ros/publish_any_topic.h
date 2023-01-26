#ifndef BEHAVIOR_TREE_PLUGIN_ROS_PUBLISH_ANY_TOPIC_H
#define BEHAVIOR_TREE_PLUGIN_ROS_PUBLISH_ANY_TOPIC_H

#include "behaviortree_cpp/action_node.h"
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_types.h>
#include <ros_babel_fish/babel_fish_message.h>

#include "behavior_tree_plugin_ros/utilities/tree_node_manager.h"

namespace behavior_tree_plugin_ros
{
class PublishAnyTopic : public BT::SyncActionNode
{
public:
  PublishAnyTopic(const std::string& name, const BT::NodeConfig& config, ros::NodeHandle& node_handle,
                  std::string topic_type);
  static BT::PortsList getPorts(std::string topic_type);
  BT::NodeStatus tick() override;
  static void Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                       ros::NodeHandle& node_handle, std::string topic_type);

private:
  std::string topic_name_;
  ros::NodeHandle& node_handle_;
  const std::string topic_type_;
  ros::Publisher publisher_;

  ros_babel_fish::BabelFish* fish_;
  TreeNodeManager* tree_node_manager_;
  ros_babel_fish::Message::Ptr message_;
  bool publish_on_change_ = false;
};

}  // namespace behavior_tree_plugin_ros
#endif  // BEHAVIOR_TREE_PLUGIN_ROS_PUBLISH_ANY_TOPIC_H
