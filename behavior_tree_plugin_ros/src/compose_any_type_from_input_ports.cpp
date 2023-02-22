#include "behavior_tree_plugin_ros/compose_any_type_from_input_ports.h"

#include "ros/callback_queue.h"

namespace behavior_tree_plugin_ros
{
ComposeAnyTypeFromInputPorts::ComposeAnyTypeFromInputPorts(const std::string& name, const BT::NodeConfig& config,
                                                           ros::NodeHandle& node_handle, std::string instance_type)
  : ComposeAnyTypeFromInputPorts(name, config, node_handle, instance_type, nullptr)

{
}

ComposeAnyTypeFromInputPorts::ComposeAnyTypeFromInputPorts(const std::string& name, const BT::NodeConfig& config,
                                                           ros::NodeHandle& node_handle, std::string instance_type,
                                                           ros_babel_fish::BabelFish* fish_ptr)
  : BT::SyncActionNode(name, config)
  , node_handle_(node_handle)
  , instance_type_(instance_type)
  , tree_node_manager_(nullptr)

{
  const auto t1 = std::chrono::system_clock::now();

  ROS_DEBUG_STREAM("fish_ptr: " << fish_ptr);
  fish_ = fish_ptr != nullptr ? fish_ptr : new ros_babel_fish::BabelFish();
  instance_ptr_ = fish_->createMessage(instance_type_);
  ROS_INFO_STREAM("ComposeAnyTypeFromInputPorts: " << instance_type_);
  
  const auto dT = (std::chrono::system_clock::now() -t1);
  const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(dT).count();
  std::cout << registrationName() << ": " << this->name() << " -> " << dt_ms << std::endl; 
}

BT::PortsList ComposeAnyTypeFromInputPorts::getPorts(std::string instance_type, ros_babel_fish::BabelFish& fish)
{
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
BT::PortsList ComposeAnyTypeFromInputPorts::getPorts(std::string service_type)
{
  ros_babel_fish::BabelFish fish;
  return getPorts(service_type, fish);
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
                                            ros::NodeHandle& node_handle, std::string instance_type,
                                            ros_babel_fish::BabelFish* fish)
{
  BT::NodeBuilder builder = [&, instance_type, fish](const std::string& name, const BT::NodeConfig config) {
    return std::make_unique<behavior_tree_plugin_ros::ComposeAnyTypeFromInputPorts>(name, config, node_handle,
                                                                                    instance_type, fish);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<ComposeAnyTypeFromInputPorts>();
  manifest.ports = fish ? ComposeAnyTypeFromInputPorts::getPorts(instance_type, *fish) :
                          ComposeAnyTypeFromInputPorts::getPorts(instance_type);
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}
void ComposeAnyTypeFromInputPorts::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                                            ros::NodeHandle& node_handle, std::string instance_type)
{
  Register(factory, registration_ID, node_handle, instance_type, nullptr);
}

}  // namespace behavior_tree_plugin_ros
