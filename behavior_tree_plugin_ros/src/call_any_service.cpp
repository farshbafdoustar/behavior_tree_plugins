#include "behavior_tree_plugin_ros/call_any_service.h"

namespace behavior_tree_plugin_ros
{
CallAnyService::CallAnyService(const std::string& name, const BT::NodeConfig& config, std::string service_type)
  : CallAnyService(name, config, service_type, nullptr)

{
}
CallAnyService::CallAnyService(const std::string& name, const BT::NodeConfig& config, std::string service_type,
                               ros_babel_fish::BabelFish* fish_ptr)
  : BT::SyncActionNode(name, config), service_type_(service_type), tree_node_manager_(nullptr)

{
  ROS_DEBUG_STREAM("fish_ptr: " << fish_ptr);
  fish_ = fish_ptr != nullptr ? fish_ptr : new ros_babel_fish::BabelFish();
  BT::Expected<std::string> service_name = getInput<std::string>("service_name");
  // Check if optional is valid. If not, throw its error
  if (!service_name)
  {
    throw BT::RuntimeError("missing required input [service_name]: ", service_name.error());
  }
  service_name_ = service_name.value();
  ROS_INFO_STREAM("CallAnyService: " << service_name_);

  BT::Expected<double> connection_timeout_ms = getInput<double>("connection_timeout_ms");
  // Check if optional is valid. If not, throw its error
  if (!connection_timeout_ms)
  {
    throw BT::RuntimeError("missing required input [connection_timeout_ms]: ", connection_timeout_ms.error());
  }
}
BT::PortsList CallAnyService::getPorts(std::string service_type, ros_babel_fish::BabelFish& fish)
{
  auto service_description_ = fish.descriptionProvider()->getServiceDescription(service_type);
  if (service_description_ == nullptr)
  {
    ROS_ERROR_STREAM("No service definition for '" << service_type << "' found!");
  }
  BT::PortsList ports;
  ports.insert(BT::InputPort<std::string>("service_name"));
  ports.insert(BT::InputPort<double>("connection_timeout_ms"));
  TreeNodeManager::makePortList(ports, BT::PortDirection::INPUT, service_description_->request->message_template,
                                "request");
  TreeNodeManager::makePortList(ports, BT::PortDirection::OUTPUT, service_description_->response->message_template,
                                "response");
  return ports;
}
BT::PortsList CallAnyService::getPorts(std::string service_type)
{
  ros_babel_fish::BabelFish fish;
  return getPorts(service_type, fish);
}
BT::NodeStatus CallAnyService::tick()
{
  ROS_DEBUG_STREAM("CallAnyService: " << service_name_);
  ros_babel_fish::Message::Ptr request = fish_->createServiceRequest(service_type_);
  if (!tree_node_manager_)
  {
    tree_node_manager_ = new TreeNodeManager(*this);
  }

  tree_node_manager_->fillMessageFromInputPorts(*request, "request");
  ros_babel_fish::TranslatedMessage::Ptr response;
  if (fish_->callService(service_name_, request, response))
  {
    tree_node_manager_->fillOutputPortsWithMessage(*response->translated_message, "response");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
void CallAnyService::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                              std::string service_type, ros_babel_fish::BabelFish* fish)
{
  BT::NodeBuilder builder = [&, service_type, fish](const std::string& name, const BT::NodeConfig config) {
    return std::make_unique<behavior_tree_plugin_ros::CallAnyService>(name, config, service_type, fish);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<CallAnyService>();

  manifest.ports = fish ? CallAnyService::getPorts(service_type, *fish) : CallAnyService::getPorts(service_type);
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}
void CallAnyService::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                              std::string service_type)
{
  Register(factory, registration_ID, service_type, nullptr);
}

}  // namespace behavior_tree_plugin_ros
