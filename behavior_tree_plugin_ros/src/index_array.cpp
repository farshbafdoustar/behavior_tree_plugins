#include "behavior_tree_plugin_ros/index_array.h"

#include "ros/callback_queue.h"

namespace behavior_tree_plugin_ros
{

IndexArray::IndexArray(const std::string& name, const BT::NodeConfig& config, ros::NodeHandle& node_handle,
                       std::string instance_type, ros_babel_fish::BabelFish* fish_ptr)
  : BT::SyncActionNode(name, config)
  , node_handle_(node_handle)
  , instance_type_(instance_type)
  , tree_node_manager_(nullptr)

{
  fish_ = fish_ptr != nullptr ? fish_ptr : new ros_babel_fish::BabelFish();
  instance_ptr_ = fish_->createMessage(instance_type_);

  ROS_INFO_STREAM("IndexArray: " << instance_type_);
}

BT::PortsList IndexArray::getPorts(std::string instance_type, ros_babel_fish::BabelFish& fish)
{
  auto message_description_ = fish.descriptionProvider()->getMessageDescription(instance_type);
  if (message_description_ == nullptr)
  {
    ROS_ERROR_STREAM("No Type definition for '" << instance_type << "' found!");
  }
  BT::PortsList ports;
  ports.insert(BT::InputPort<uint64_t>("index_"));
  TreeNodeManager::makePortForInstanceIndexAt(ports, BT::PortDirection::INPUT, message_description_->message_template,
                                              "instance_");
  TreeNodeManager::makePortForResultIndexAt(ports, BT::PortDirection::OUTPUT, message_description_->message_template,
                                            "result");
  return ports;
}
BT::PortsList IndexArray::getPorts(std::string instance_type)
{
  ros_babel_fish::BabelFish fish;
  return getPorts(instance_type, fish);
}
BT::NodeStatus IndexArray::tick()
{
  ROS_DEBUG_STREAM("IndexArray: " << instance_type_);

  if (!tree_node_manager_)
  {
    tree_node_manager_ = new TreeNodeManager(*this);
  }
  BT::Expected<uint64_t> index = getInput<uint64_t>("index_");

  if (!index)
  {
    throw BT::RuntimeError("missing required input [index_]: ", index.error());
  }
  auto message_description_ = fish_->descriptionProvider()->getMessageDescription(instance_type_);
  if (message_description_ == nullptr)
  {
    ROS_ERROR_STREAM("No Type definition for '" << instance_type_ << "' found!");
  }
  tree_node_manager_->fillResultWithIndexAt(message_description_->message_template, "instance_", "result",
                                            index.value());

  return BT::NodeStatus::SUCCESS;
}

void IndexArray::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                          ros::NodeHandle& node_handle, std::string instance_type, ros_babel_fish::BabelFish* fish)
{
  BT::NodeBuilder builder = [&, instance_type, fish](const std::string& name, const BT::NodeConfig config) {
    return std::make_unique<behavior_tree_plugin_ros::IndexArray>(name, config, node_handle, instance_type, fish);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<IndexArray>();
  manifest.ports = fish ? IndexArray::getPorts(instance_type, *fish) : IndexArray::getPorts(instance_type);
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}

}  // namespace behavior_tree_plugin_ros
