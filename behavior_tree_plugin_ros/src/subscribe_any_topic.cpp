#include "behavior_tree_plugin_ros/subscribe_any_topic.h"

#include "ros/callback_queue.h"

namespace behavior_tree_plugin_ros
{
SubscribeAnyTopic::SubscribeAnyTopic(const std::string& name, const BT::NodeConfig& config,
                                     ros::NodeHandle& node_handle, std::string topic_type)
  : SubscribeAnyTopic(name, config, node_handle, topic_type, nullptr)
{
}
SubscribeAnyTopic::SubscribeAnyTopic(const std::string& name, const BT::NodeConfig& config,
                                     ros::NodeHandle& node_handle, std::string topic_type,
                                     ros_babel_fish::BabelFish* fish_ptr)
  : BT::SyncActionNode(name, config), node_handle_(node_handle), topic_type_(topic_type), tree_node_manager_(nullptr)

{
  ROS_DEBUG_STREAM("fish_ptr: " << fish_ptr);
  fish_ = fish_ptr != nullptr ? fish_ptr : new ros_babel_fish::BabelFish();
  BT::Expected<std::string> topic_name = getInput<std::string>("topic_name");

  if (!topic_name)
  {
    throw BT::RuntimeError("missing required input [topic_name]: ", topic_name.error());
  }
  topic_name_ = topic_name.value();

  BT::Expected<bool> is_statefull = getInput<bool>("is_statefull");

  if (is_statefull)
  {
    is_statefull_ = is_statefull.value();
  }

  BT::Expected<bool> activate_callback_in_tick = getInput<bool>("activate_callback_in_tick");

  if (activate_callback_in_tick)
  {
    activate_callback_in_tick_ = activate_callback_in_tick.value();
  }

  ROS_INFO_STREAM("SubscribeAnyTopic: " << topic_name_);
  subscriber_ = node_handle_.subscribe(topic_name_, 1, &SubscribeAnyTopic::callback, this);
}

BT::PortsList SubscribeAnyTopic::getPorts(std::string topic_type, ros_babel_fish::BabelFish& fish)
{
  auto message_description_ = fish.descriptionProvider()->getMessageDescription(topic_type);
  if (message_description_ == nullptr)
  {
    ROS_ERROR_STREAM("No Type definition for '" << topic_type << "' found!");
  }
  BT::PortsList ports;
  ports.insert(BT::InputPort<std::string>("topic_name"));
  ports.insert(BT::InputPort<bool>("is_statefull"));
  ports.insert(BT::InputPort<bool>("activate_callback_in_tick"));

  TreeNodeManager::makePortList(ports, BT::PortDirection::OUTPUT, message_description_->message_template, "");
  return ports;
}
BT::PortsList SubscribeAnyTopic::getPorts(std::string topic_type)
{
  ros_babel_fish::BabelFish fish;
  return getPorts(topic_type, fish);
}
BT::NodeStatus SubscribeAnyTopic::tick()
{
  if(activate_callback_in_tick_)
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
  }
  if (is_statefull_)
  {
    if (message_)
    {
      ros_babel_fish::TranslatedMessage::Ptr translated = fish_->translateMessage(message_);
      if (!tree_node_manager_)
      {
        tree_node_manager_ = new TreeNodeManager(*this);
      }
      tree_node_manager_->fillOutputPortsWithMessage(*translated->translated_message, "");
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      if (!is_statefull_error_shown)
      {
        ROS_WARN_STREAM("Is Latch Enabled on Publisher? Waiting for Message on Topic: " << topic_name_
                                                                                        << ". This message will be "
                                                                                           "shown only once.");
        is_statefull_error_shown = true;
      }
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    if (!message_)
    {
      // this is called one time in while in main node of Bt
      //  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
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
  return BT::NodeStatus::FAILURE;
}

void SubscribeAnyTopic::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                                 ros::NodeHandle& node_handle, std::string topic_type, ros_babel_fish::BabelFish* fish)
{
  BT::NodeBuilder builder = [&, topic_type, fish](const std::string& name, const BT::NodeConfig config) {
    return std::make_unique<behavior_tree_plugin_ros::SubscribeAnyTopic>(name, config, node_handle, topic_type, fish);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<SubscribeAnyTopic>();
  manifest.ports = fish ? SubscribeAnyTopic::getPorts(topic_type, *fish) : SubscribeAnyTopic::getPorts(topic_type);
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}
void SubscribeAnyTopic::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                                 ros::NodeHandle& node_handle, std::string topic_type)
{
  Register(factory, registration_ID, node_handle, topic_type, nullptr);
}

void SubscribeAnyTopic::callback(const ros_babel_fish::BabelFishMessage::ConstPtr& message)
{
  message_ = message;
}

}  // namespace behavior_tree_plugin_ros
