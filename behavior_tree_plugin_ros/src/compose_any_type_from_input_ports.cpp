#include "behavior_tree_plugin_ros/compose_any_type_from_input_ports.h"

#include "ros/callback_queue.h"

namespace behavior_tree_plugin_ros
{
ComposeAnyTypeFromInputPorts::ComposeAnyTypeFromInputPorts(const std::string& name, const BT::NodeConfig& config,
                                                           ros::NodeHandle& node_handle, std::string instance_type)
  : BT::SyncActionNode(name, config)
  , node_handle_(node_handle)
  , instance_type_(instance_type)
  , fish_(new ros_babel_fish::BabelFish())
  , tree_node_manager_(nullptr)

{
  instance_ptr_ = fish_->createMessage(instance_type_);
  ROS_INFO_STREAM("ComposeAnyTypeFromInputPorts: " << instance_type_);
}

BT::PortsList ComposeAnyTypeFromInputPorts::getPorts(std::string instance_type)
{
  ros_babel_fish::BabelFish fish;
  auto message_description_ = fish.descriptionProvider()->getMessageDescription(instance_type);
  if (message_description_ == nullptr)
  {
    ROS_ERROR_STREAM("No Type definition for '" << instance_type << "' found!");
  }
  BT::PortsList ports;
  ports.insert(BT::OutputPort<ros_babel_fish::BabelFishMessage>("instance_"));

  TreeNodeManager::makePortList(ports, BT::PortDirection::INPUT, message_description_->message_template, "");
  return ports;
}
BT::NodeStatus ComposeAnyTypeFromInputPorts::tick()
{
  ROS_DEBUG_STREAM("ComposeAnyTypeFromInputPorts: " << instance_type_);

  if (!tree_node_manager_)
  {
    tree_node_manager_ = new TreeNodeManager(*this);
  }
  bool changed = tree_node_manager_->fillMessageFromInputPorts(*instance_ptr_, "");
  ros_babel_fish::BabelFishMessage::Ptr translated_message_ptr = fish_->translateMessage(instance_ptr_);
  this->setOutput<ros_babel_fish::BabelFishMessage>("instance_", *translated_message_ptr);

  return BT::NodeStatus::SUCCESS;
}

void ComposeAnyTypeFromInputPorts::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                                            ros::NodeHandle& node_handle, std::string instance_type)
{
  BT::NodeBuilder builder = [&, instance_type, &node_handle](const std::string& name, const BT::NodeConfig config) {
    return std::make_unique<behavior_tree_plugin_ros::ComposeAnyTypeFromInputPorts>(name, config, node_handle,
                                                                                    instance_type);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<ComposeAnyTypeFromInputPorts>();
  manifest.ports = ComposeAnyTypeFromInputPorts::getPorts(instance_type);
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}

}  // namespace behavior_tree_plugin_ros
