#include "behavior_tree_plugin_ros/decompose_any_type_to_output_ports.h"

#include "ros/callback_queue.h"

namespace behavior_tree_plugin_ros
{
DecomposeAnyTypeToOutputPorts::DecomposeAnyTypeToOutputPorts(const std::string& name, const BT::NodeConfig& config,
                                                             ros::NodeHandle& node_handle, std::string instance_type)
  : DecomposeAnyTypeToOutputPorts(name, config, node_handle, instance_type, nullptr)
{
}
DecomposeAnyTypeToOutputPorts::DecomposeAnyTypeToOutputPorts(const std::string& name, const BT::NodeConfig& config,
                                                             ros::NodeHandle& node_handle, std::string instance_type,
                                                             ros_babel_fish::BabelFish* fish_ptr)
  : BT::SyncActionNode(name, config)
  , node_handle_(node_handle)
  , instance_type_(instance_type)
  , tree_node_manager_(nullptr)

{
  ROS_DEBUG_STREAM("fish_ptr: " << fish_ptr);
  fish_ = fish_ptr != nullptr ? fish_ptr : new ros_babel_fish::BabelFish();
  ROS_INFO_STREAM("DecomposeAnyTypeToOutputPorts: " << instance_type_);
}

BT::PortsList DecomposeAnyTypeToOutputPorts::getPorts(std::string instance_type, ros_babel_fish::BabelFish& fish)
{
  auto message_description_ = fish.descriptionProvider()->getMessageDescription(instance_type);
  if (message_description_ == nullptr)
  {
    ROS_ERROR_STREAM("No Type definition for '" << instance_type << "' found!");
  }
  BT::PortsList ports;
  ports.insert(BT::InputPort<ros_babel_fish::BabelFishMessage>("instance_"));

  TreeNodeManager::makePortList(ports, BT::PortDirection::OUTPUT, message_description_->message_template, "");
  return ports;
}
BT::PortsList DecomposeAnyTypeToOutputPorts::getPorts(std::string instance_type)
{
  ros_babel_fish::BabelFish fish;
  return getPorts(instance_type, fish);
}
BT::NodeStatus DecomposeAnyTypeToOutputPorts::tick()
{
  ROS_DEBUG_STREAM("DecomposeAnyTypeToOutputPorts: " << instance_type_);
  BT::Expected<ros_babel_fish::BabelFishMessage> instance = getInput<ros_babel_fish::BabelFishMessage>("instance_");

  if (!instance)
  {
    throw BT::RuntimeError("missing required input [instance_]: ", instance.error());
  }

  ros_babel_fish::BabelFishMessage temp = instance.value();
  ros_babel_fish::BabelFishMessage::Ptr instance_ptr(&temp);

  ros_babel_fish::TranslatedMessage::Ptr translated = fish_->translateMessage(instance_ptr);
  if (!tree_node_manager_)
  {
    tree_node_manager_ = new TreeNodeManager(*this);
  }
  tree_node_manager_->fillOutputPortsWithMessage(*translated->translated_message, "");
  return BT::NodeStatus::SUCCESS;
}

void DecomposeAnyTypeToOutputPorts::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                                             ros::NodeHandle& node_handle, std::string instance_type,
                                             ros_babel_fish::BabelFish* fish)
{
  BT::NodeBuilder builder = [&, instance_type, fish](const std::string& name, const BT::NodeConfig config) {
    return std::make_unique<behavior_tree_plugin_ros::DecomposeAnyTypeToOutputPorts>(name, config, node_handle,
                                                                                     instance_type, fish);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<DecomposeAnyTypeToOutputPorts>();
  manifest.ports = fish ? DecomposeAnyTypeToOutputPorts::getPorts(instance_type, *fish) :
                          DecomposeAnyTypeToOutputPorts::getPorts(instance_type);
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}
void DecomposeAnyTypeToOutputPorts::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                                             ros::NodeHandle& node_handle, std::string instance_type)
{
  Register(factory, registration_ID, node_handle, instance_type, nullptr);
}

}  // namespace behavior_tree_plugin_ros
