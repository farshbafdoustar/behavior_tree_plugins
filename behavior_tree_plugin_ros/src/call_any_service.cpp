#include "behavior_tree_plugin_ros/call_any_service.h"

#include "behavior_tree_plugin_ros/utilities/tree_node_manager.h"

namespace behavior_tree_plugin_ros
{
CallAnyService::CallAnyService(const std::string& name, const BT::NodeConfig& config, std::string service_type)
  : BT::SyncActionNode(name, config), service_type_(service_type)

{
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

BT::PortsList CallAnyService::getPorts(std::string service_type)
{
  ros_babel_fish::BabelFish fish_;
  auto service_description_ = fish_.descriptionProvider()->getServiceDescription(service_type);
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
BT::NodeStatus CallAnyService::tick()
{
  ros_babel_fish::BabelFish fish;
  ros_babel_fish::Message::Ptr request = fish.createServiceRequest(service_type_);
  TreeNodeManager tree_node_manager_(*this);
  ROS_DEBUG_STREAM("CallAnyService: " << service_name_);

  tree_node_manager_.fillMessageFromInputPorts(*request, "request");

  ros_babel_fish::TranslatedMessage::Ptr response;
  if (fish.callService(service_name_, request, response))
  {
    tree_node_manager_.fillOutputPortsWithMessage(*response->translated_message, "response");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
void CallAnyService::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                              std::string service_type)
{
  BT::NodeBuilder builder = [&, service_type](const std::string& name, const BT::NodeConfig config) {
    return std::make_unique<behavior_tree_plugin_ros::CallAnyService>(name, config, service_type);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<CallAnyService>();
  manifest.ports = CallAnyService::getPorts(service_type);
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}

}  // namespace behavior_tree_plugin_ros
