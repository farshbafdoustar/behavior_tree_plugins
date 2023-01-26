#include "behavior_tree_plugin_ros/publish_any_topic.h"

#include "ros/callback_queue.h"

namespace behavior_tree_plugin_ros
{
PublishAnyTopic::PublishAnyTopic(const std::string& name, const BT::NodeConfig& config, ros::NodeHandle& node_handle,
                                 std::string topic_type)
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
  BT::Expected<bool> is_latched = getInput<bool>("is_latched");
  bool latched = false;
  if (is_latched)
  {
    latched = is_latched.value();
  }

  topic_name_ = topic_name.value();
  publisher_ = fish_->advertise(node_handle_, topic_type_, topic_name_, 1, latched);
  message_ = fish_->createMessage(topic_type_);
  ROS_INFO_STREAM("PublishAnyTopic: " << topic_name_);
}

BT::PortsList PublishAnyTopic::getPorts(std::string topic_type)
{
  ros_babel_fish::BabelFish fish;
  auto message_description_ = fish.descriptionProvider()->getMessageDescription(topic_type);
  if (message_description_ == nullptr)
  {
    ROS_ERROR_STREAM("No Topic definition for '" << topic_type << "' found!");
  }
  BT::PortsList ports;
  ports.insert(BT::InputPort<std::string>("topic_name"));
  ports.insert(BT::InputPort<bool>("is_latched"));

  TreeNodeManager::makePortList(ports, BT::PortDirection::INPUT, message_description_->message_template, "");
  return ports;
}
BT::NodeStatus PublishAnyTopic::tick()
{
  ROS_DEBUG_STREAM("PublishAnyTopic: " << topic_name_);

  if (!tree_node_manager_)
  {
    tree_node_manager_ = new TreeNodeManager(*this);
  }
  if (tree_node_manager_->fillMessageFromInputPorts(*message_, ""))
  {
    ros_babel_fish::BabelFishMessage::Ptr translated_message = fish_->translateMessage(message_);
    publisher_.publish(translated_message);
  }
  return BT::NodeStatus::SUCCESS;
}

void PublishAnyTopic::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                               ros::NodeHandle& node_handle, std::string topic_type)
{
  BT::NodeBuilder builder = [&, topic_type, &node_handle](const std::string& name, const BT::NodeConfig config) {
    return std::make_unique<behavior_tree_plugin_ros::PublishAnyTopic>(name, config, node_handle, topic_type);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<PublishAnyTopic>();
  manifest.ports = PublishAnyTopic::getPorts(topic_type);
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}

}  // namespace behavior_tree_plugin_ros
