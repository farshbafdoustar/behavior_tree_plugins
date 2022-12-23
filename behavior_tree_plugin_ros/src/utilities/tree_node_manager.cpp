#include "behavior_tree_plugin_ros/utilities/tree_node_manager.h"

namespace behavior_tree_plugin_ros
{
TreeNodeManager::TreeNodeManager(BT::TreeNode& tree_node) : tree_node_(tree_node)

{
}

void TreeNodeManager::makePortList(BT::PortsList& local_port_list, const BT::PortDirection& port_direction,
                                   const ros_babel_fish::MessageTemplate::ConstPtr message_template,
                                   const std::string& prefix)
{
  if (message_template->type == ros_babel_fish::MessageTypes::Compound)
  {
    for (size_t i = 0; i < message_template->compound.names.size(); ++i)
    {
      std::string name = message_template->compound.names[i];
      ROS_DEBUG_STREAM("Port created: " << (prefix == "" ? name : prefix + "." + name));

      makePortList(local_port_list, port_direction, message_template->compound.types[i],
                   prefix == "" ? name : prefix + "." + name);
    }
  }
  else if (message_template->type == ros_babel_fish::MessageTypes::Array)
  {
    auto& base = message_template->array;
    switch (base.element_type)
    {
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool:
        local_port_list.insert(BT::CreatePort<std::vector<bool>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        local_port_list.insert(BT::CreatePort<std::vector<uint8_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        local_port_list.insert(BT::CreatePort<std::vector<uint16_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        local_port_list.insert(BT::CreatePort<std::vector<uint32_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        local_port_list.insert(BT::CreatePort<std::vector<uint64_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int8:
        local_port_list.insert(BT::CreatePort<std::vector<int8_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int16:
        local_port_list.insert(BT::CreatePort<std::vector<int16_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int32:
        local_port_list.insert(BT::CreatePort<std::vector<int32_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int64:
        local_port_list.insert(BT::CreatePort<std::vector<int64_t>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Float32:
        local_port_list.insert(BT::CreatePort<std::vector<float>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Float64:
        local_port_list.insert(BT::CreatePort<std::vector<double>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Time:
        ROS_ERROR_STREAM("Port of type Time ARRAY NOT implemented yet;Skipping..." << prefix);
        // local_port_list.insert(BT::CreatePort<std::vector<ros::Time>>(port_direction,prefix));
        break;
      case ros_babel_fish::MessageTypes::Duration:
        ROS_ERROR_STREAM("Port of type Duration ARRAY NOT implemented yet;Skipping..." << prefix);
        // local_port_list.insert(BT::CreatePort<std::vector<ros::Duration>>(port_direction,prefix));
        break;
      case ros_babel_fish::MessageTypes::String:
        local_port_list.insert(BT::CreatePort<std::vector<std::string>>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::Array:  // Arrays of arrays are actually not supported in the ROS msg format
      {
        ROS_ERROR_STREAM("Array Type ports NOT implemented yet; Skipping" << prefix);
        // for (size_t i = 0; i < base.length; ++i)
        // {
        //   makePortList(local_port_list, port_direction, message_template->compound.types[i], prefix);
        // }
        break;
      }
    }
  }
  else
  {
    switch (message_template->type)
    {
      case ros_babel_fish::MessageTypes::Array:
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool:
        local_port_list.insert(BT::CreatePort<bool>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        local_port_list.insert(BT::CreatePort<uint8_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        local_port_list.insert(BT::CreatePort<uint16_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        local_port_list.insert(BT::CreatePort<uint32_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        local_port_list.insert(BT::CreatePort<uint64_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int8:
        local_port_list.insert(BT::CreatePort<int8_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int16:
        local_port_list.insert(BT::CreatePort<int16_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int32:
        local_port_list.insert(BT::CreatePort<int32_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Int64:
        local_port_list.insert(BT::CreatePort<int64_t>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Float32:
        local_port_list.insert(BT::CreatePort<float>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Float64:
        local_port_list.insert(BT::CreatePort<double>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Time:
        local_port_list.insert(BT::CreatePort<ros::Time>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::Duration:
        local_port_list.insert(BT::CreatePort<ros::Duration>(port_direction, prefix));
        break;
      case ros_babel_fish::MessageTypes::String:
        local_port_list.insert(BT::CreatePort<std::string>(port_direction, prefix));
        break;
    }
  }
}
void TreeNodeManager::fillMessageFromInputPorts(ros_babel_fish::Message& request, const std::string& prefix)
{
  if (request.type() == ros_babel_fish::MessageTypes::Compound)
  {
    auto& compound = request.as<ros_babel_fish::CompoundMessage>();
    for (size_t i = 0; i < compound.keys().size(); ++i)
    {
      std::string name = compound.keys()[i];
      // ROS_DEBUG_STREAM("Filling message from input port: " << (prefix == "" ? name : prefix + "." + name));

      fillMessageFromInputPorts(compound[name], prefix == "" ? name : prefix + "." + name);
    }
  }
  else if (request.type() == ros_babel_fish::MessageTypes::Array)
  {
    auto& base = request.as<ros_babel_fish::ArrayMessageBase>();
    switch (base.elementType())
    {
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        ROS_ERROR_STREAM("Filling UInt8 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        ROS_ERROR_STREAM("Filling UInt16 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        ROS_ERROR_STREAM("Filling UInt32 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        ROS_ERROR_STREAM("Filling UInt64 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int8:
        ROS_ERROR_STREAM("Filling Int8 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int16:
        ROS_ERROR_STREAM("Filling Int16 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int32:
        ROS_ERROR_STREAM("Filling Int32 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int64:
        ROS_ERROR_STREAM("Filling Int64 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Float32:
        ROS_ERROR_STREAM("Filling Float32 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Float64:
        ROS_ERROR_STREAM("Filling Float64 Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Time:
        ROS_ERROR_STREAM("Filling Time Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Duration:
        ROS_ERROR_STREAM("Filling Duration Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::String: {
        auto input = tree_node_.getInput<std::vector<std::string>>(prefix);
        ROS_DEBUG_STREAM("request string 111:: " + prefix);

        if (input)
        {
          for (std::string str : input.value())
          {
            ROS_DEBUG_STREAM("request string:: " + prefix << " : " << str);
            request.as<ros_babel_fish::ArrayMessage<std::string>>().push_back(str);
          }
        }

        break;
      }
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::Array:  // Arrays of arrays are actually not supported in the ROS msg format
        ROS_ERROR_STREAM("Filling Array of Arrays or Compound fields NOT implemented yet");
        // {
        //   std::cout << std::endl;
        //    auto &array = base.as<ros_babel_fish::ArrayMessage<ros_babel_fish::Message>>();
        //   for ( size_t i = 0; i < array.length(); ++i )
        //   {
        //     std::cout << prefix << "- ";
        //     makeRequest(array[i], prefix );

        //   }
        //   break;
        // }
        break;
    }
  }
  else
  {
    switch (request.type())
    {
      case ros_babel_fish::MessageTypes::Array:
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool: {
        auto input = tree_node_.getInput<bool>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<bool>());
        break;
      }
      case ros_babel_fish::MessageTypes::UInt8: {
        auto input = tree_node_.getInput<uint8_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<uint8_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::UInt16: {
        auto input = tree_node_.getInput<uint16_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<uint16_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::UInt32: {
        auto input = tree_node_.getInput<uint32_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<uint32_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::UInt64: {
        auto input = tree_node_.getInput<uint64_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<uint64_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::Int8: {
        auto input = tree_node_.getInput<int8_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<int8_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::Int16: {
        auto input = tree_node_.getInput<int16_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<int16_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::Int32: {
        auto input = tree_node_.getInput<int32_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<int32_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::Int64: {
        auto input = tree_node_.getInput<int64_t>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<int64_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::Float32: {
        auto input = tree_node_.getInput<float>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<float>());
        break;
      }
      case ros_babel_fish::MessageTypes::Float64: {
        auto input = tree_node_.getInput<double>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<double>());
        break;
      }
      case ros_babel_fish::MessageTypes::Time: {
        ROS_ERROR_STREAM("Filling Time fields NOT implemented yet");
        // auto input = tree_node_.getInput<ros::Time>(prefix);
        // if (input)
        // {
        //   request = input.value();
        // }
        break;
      }
      case ros_babel_fish::MessageTypes::Duration: {
        ROS_ERROR_STREAM("Filling Duration fields NOT implemented yet");
        // auto input = tree_node_.getInput<ros::Duration>(prefix);
        // if (input)
        // {
        //   request = input.value();
        // }
        break;
      }
      case ros_babel_fish::MessageTypes::String: {
        auto input = tree_node_.getInput<std::string>(prefix);
        if (input)
        {
          request = input.value();
        }
        ROS_DEBUG_STREAM(prefix << " : " << request.value<std::string>());
        break;
      }
    }
  }
}
void TreeNodeManager::fillOutputPortsWithMessage(ros_babel_fish::Message& response, const std::string& prefix)
{
  if (response.type() == ros_babel_fish::MessageTypes::Compound)
  {
    auto& compound = response.as<ros_babel_fish::CompoundMessage>();
    for (size_t i = 0; i < compound.keys().size(); ++i)
    {
      std::string name = compound.keys()[i];
      // ROS_DEBUG_STREAM("Filling output ports with message: " << (prefix == "" ? name : prefix + "." + name));

      fillOutputPortsWithMessage(compound[name], prefix == "" ? name : prefix + "." + name);
    }
  }
  else if (response.type() == ros_babel_fish::MessageTypes::Array)
  {
    auto& base = response.as<ros_babel_fish::ArrayMessageBase>();
    switch (base.elementType())
    {
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int8:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int16:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int32:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Int64:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Float32:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Float64:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Time:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Duration:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::String:
        ROS_ERROR_STREAM("Filling Bool Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::Array:  // Arrays of arrays are actually not supported in the ROS msg format
      {
        ROS_ERROR_STREAM("Filling Array of Arrays fields NOT implemented yet");
        break;
      }
    }
  }
  else
  {
    switch (response.type())
    {
      case ros_babel_fish::MessageTypes::Array:
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::None:
        ROS_DEBUG_STREAM(prefix << " is Array/Compound/None");
        break;
      case ros_babel_fish::MessageTypes::Bool:
        tree_node_.setOutput<bool>(prefix, response.value<bool>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<bool>());
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        tree_node_.setOutput<uint8_t>(prefix, response.value<uint8_t>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<uint8_t>());
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        tree_node_.setOutput<uint16_t>(prefix, response.value<uint16_t>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<uint16_t>());
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        tree_node_.setOutput<uint32_t>(prefix, response.value<uint32_t>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<uint32_t>());
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        tree_node_.setOutput<uint64_t>(prefix, response.value<uint64_t>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<uint64_t>());
        break;
      case ros_babel_fish::MessageTypes::Int8:
        tree_node_.setOutput<int8_t>(prefix, response.value<int8_t>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<int8_t>());
        break;
      case ros_babel_fish::MessageTypes::Int16:
        tree_node_.setOutput<int16_t>(prefix, response.value<int16_t>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<int16_t>());
        break;
      case ros_babel_fish::MessageTypes::Int32:
        tree_node_.setOutput<int32_t>(prefix, response.value<int32_t>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<int32_t>());
        break;
      case ros_babel_fish::MessageTypes::Int64:
        tree_node_.setOutput<int64_t>(prefix, response.value<int64_t>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<int64_t>());
        break;
      case ros_babel_fish::MessageTypes::Float32:
        tree_node_.setOutput<float>(prefix, response.value<float>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<float>());
        break;
      case ros_babel_fish::MessageTypes::Float64:
        tree_node_.setOutput<double>(prefix, response.value<double>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<double>());
        break;
      case ros_babel_fish::MessageTypes::Time:
        tree_node_.setOutput<ros::Time>(prefix, response.value<ros::Time>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<ros::Time>());
        break;
      case ros_babel_fish::MessageTypes::Duration:
        tree_node_.setOutput<ros::Duration>(prefix, response.value<ros::Duration>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<ros::Duration>());
        break;
      case ros_babel_fish::MessageTypes::String:
        tree_node_.setOutput<std::string>(prefix, response.value<std::string>());
        ROS_DEBUG_STREAM(prefix << " : " << response.value<std::string>());
        break;
    }
  }
}

}  // namespace behavior_tree_plugin_ros

namespace BT
{
template <>
inline std::vector<std::string> convertFromString(StringView str)
{
  std::vector<std::string> tockens;
  if (!str.empty())
  {
    std::istringstream str_stream(std::string(str), std::ios_base::in);
    std::string tocken;
    while (std::getline(str_stream, tocken, ';'))
    {
      if (tocken != "")
      {
        tockens.push_back(name);
      }
    }
  }

  return tockens;
}
}  // end namespace BT
