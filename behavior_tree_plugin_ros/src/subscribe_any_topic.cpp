#include "behavior_tree_plugin_ros/subscribe_any_topic.h"

#include "ros/callback_queue.h"

namespace behavior_tree_plugin_ros
{
SubscribeAnyTopic::SubscribeAnyTopic(const std::string& name, const BT::NodeConfig& config,
                                     ros::NodeHandle& node_handle, std::string topic_type)
  : BT::SyncActionNode(name, config)
  , node_handle_(node_handle)
  , topic_type_(topic_type)
  , fish_(new ros_babel_fish::BabelFish())
  , tree_node_manager_(nullptr)

{
  BT::Expected<std::string> topic_name = getInput<std::string>("topic_name");

  if (!topic_name)
  {
    throw BT::RuntimeError("missing required input [topic_name]: ", topic_name.error());
  }
  topic_name_ = topic_name.value();
  ROS_INFO_STREAM("SubscribeAnyTopic: " << topic_name_);
  subscriber_ = node_handle_.subscribe(topic_name_, 1, &SubscribeAnyTopic::callback, this);
}

BT::PortsList SubscribeAnyTopic::getPorts(std::string topic_type)
{
  ros_babel_fish::BabelFish fish;
  auto message_description_ = fish.descriptionProvider()->getMessageDescription(topic_type);
  if (message_description_ == nullptr)
  {
    ROS_ERROR_STREAM("No Topic definition for '" << topic_type << "' found!");
  }
  BT::PortsList ports;
  ports.insert(BT::InputPort<std::string>("topic_name"));

  TreeNodeManager::makePortList(ports, BT::PortDirection::OUTPUT, message_description_->message_template, "");
  return ports;
}
BT::NodeStatus SubscribeAnyTopic::tick()
{
  if (!message_)
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    ROS_DEBUG_STREAM("SubscribeAnyTopic: " << topic_name_);
    ros_babel_fish::TranslatedMessage::Ptr translated = fish_->translateMessage(message_);
    if (!tree_node_manager_)
    {
      tree_node_manager_ = new TreeNodeManager(*this);
    }
    tree_node_manager_->fillOutputPortsWithMessage(*translated->translated_message, "");
    message_ = nullptr;
    return BT::NodeStatus::SUCCESS;
  }
}

void SubscribeAnyTopic::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                                 ros::NodeHandle& node_handle, std::string topic_type)
{
  BT::NodeBuilder builder = [&, topic_type, &node_handle](const std::string& name, const BT::NodeConfig config) {
    return std::make_unique<behavior_tree_plugin_ros::SubscribeAnyTopic>(name, config, node_handle, topic_type);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<SubscribeAnyTopic>();
  manifest.ports = SubscribeAnyTopic::getPorts(topic_type);
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}

void SubscribeAnyTopic::callback(const ros_babel_fish::BabelFishMessage::ConstPtr& message)
{
  message_ = message;
}

}  // namespace behavior_tree_plugin_ros
