#include "behavior_tree_plugin_ros/utilities/tree_node_manager.h"

#include <charconv>

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
      if (name == "name" || name == "ID")
      {
        name = name + "_";
      }
      ROS_DEBUG_STREAM("Ports will be created as children for: " << (prefix == "" ? name : prefix + "." + name));

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
bool TreeNodeManager::fillMessageFromInputPorts(ros_babel_fish::Message& message, const std::string& prefix)
{
  bool is_message_updated = false;
  if (message.type() == ros_babel_fish::MessageTypes::Compound)
  {
    auto& compound = message.as<ros_babel_fish::CompoundMessage>();
    for (size_t i = 0; i < compound.keys().size(); ++i)
    {
      std::string name = compound.keys()[i];
      // ROS_DEBUG_STREAM("Filling message from input port: " << (prefix == "" ? name : prefix + "." + name));

      is_message_updated |= fillMessageFromInputPorts(compound[name], prefix == "" ? name : prefix + "." + name);
    }
  }
  else if (message.type() == ros_babel_fish::MessageTypes::Array)
  {
    auto& base = message.as<ros_babel_fish::ArrayMessageBase>();
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
        ROS_DEBUG_STREAM("message string 111:: " + prefix);
        auto old_message = message.as<ros_babel_fish::ArrayMessage<std::string>>().clone();
        message = ros_babel_fish::ArrayMessage<std::string>(ros_babel_fish::MessageTypes::String);
        if (input)
        {
          for (std::string str : input.value())
          {
            ROS_DEBUG_STREAM("message string:: " + prefix << " : " << str);
            message.as<ros_babel_fish::ArrayMessage<std::string>>().push_back(str);
          }
        }
        if (old_message->as<ros_babel_fish::ArrayMessage<std::string>>().length() ==
            message.as<ros_babel_fish::ArrayMessage<std::string>>().length())
        {
          for (int i = 0; i < old_message->as<ros_babel_fish::ArrayMessage<std::string>>().length(); i++)
          {
            if (old_message->as<ros_babel_fish::ArrayMessage<std::string>>()[i] !=
                message.as<ros_babel_fish::ArrayMessage<std::string>>()[i])
            {
              is_message_updated |= true;
              break;
            }
          }
        }
        else
        {
          is_message_updated |= true;
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
    switch (message.type())
    {
      case ros_babel_fish::MessageTypes::Array:
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool: {
        auto input = tree_node_.getInput<bool>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = false;
        }
        is_message_updated |= (old_message->value<bool>() != message.value<bool>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<bool>());
        break;
      }
      case ros_babel_fish::MessageTypes::UInt8: {
        auto input = tree_node_.getInput<uint8_t>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = 0;
        }
        is_message_updated |= (old_message->value<uint8_t>() != message.value<uint8_t>());

        ROS_DEBUG_STREAM(prefix << " : " << message.value<uint8_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::UInt16: {
        auto input = tree_node_.getInput<uint16_t>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = 0;
        }
        is_message_updated |= (old_message->value<uint16_t>() != message.value<uint16_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<uint16_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::UInt32: {
        auto input = tree_node_.getInput<uint32_t>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = 0;
        }
        is_message_updated |= (old_message->value<uint32_t>() != message.value<uint32_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<uint32_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::UInt64: {
        auto input = tree_node_.getInput<uint64_t>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = 0;
        }
        is_message_updated |= (old_message->value<uint64_t>() != message.value<uint64_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<uint64_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::Int8: {
        auto input = tree_node_.getInput<int8_t>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = 0;
        }
        is_message_updated |= (old_message->value<int8_t>() != message.value<int8_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<int8_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::Int16: {
        auto input = tree_node_.getInput<int16_t>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = 0;
        }
        is_message_updated |= (old_message->value<int16_t>() != message.value<int16_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<int16_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::Int32: {
        auto input = tree_node_.getInput<int32_t>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = 0;
        }
        is_message_updated |= (old_message->value<int32_t>() != message.value<int32_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<int32_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::Int64: {
        auto input = tree_node_.getInput<int64_t>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = 0;
        }
        is_message_updated |= (old_message->value<int64_t>() != message.value<int64_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<int64_t>());
        break;
      }
      case ros_babel_fish::MessageTypes::Float32: {
        auto input = tree_node_.getInput<float>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = 0;
        }
        is_message_updated |= (old_message->value<float>() != message.value<float>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<float>());
        break;
      }
      case ros_babel_fish::MessageTypes::Float64: {
        auto input = tree_node_.getInput<double>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = 0;
        }
        is_message_updated |= (old_message->value<double>() != message.value<double>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<double>());
        break;
      }
      case ros_babel_fish::MessageTypes::Time: {
        ROS_ERROR_STREAM("Filling Time fields NOT implemented yet");
        // auto input = tree_node_.getInput<ros::Time>(prefix);
        // if (input)
        // {
        //   message = input.value();
        // }
        break;
      }
      case ros_babel_fish::MessageTypes::Duration: {
        ROS_ERROR_STREAM("Filling Duration fields NOT implemented yet");
        // auto input = tree_node_.getInput<ros::Duration>(prefix);
        // if (input)
        // {
        //   message = input.value();
        // }
        break;
      }
      case ros_babel_fish::MessageTypes::String: {
        auto input = tree_node_.getInput<std::string>(prefix);
        auto old_message = message.clone();
        if (input)
        {
          message = input.value();
        }
        else
        {
          message = "";
        }
        is_message_updated |= (old_message->value<std::string>() != message.value<std::string>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<std::string>());
        break;
      }
    }
  }
  return is_message_updated;
}
void TreeNodeManager::fillOutputPortsWithMessage(const ros_babel_fish::Message& message, const std::string& prefix)
{
  if (message.type() == ros_babel_fish::MessageTypes::Compound)
  {
    auto& compound = message.as<ros_babel_fish::CompoundMessage>();
    for (size_t i = 0; i < compound.keys().size(); ++i)
    {
      std::string name = compound.keys()[i];
      // ROS_DEBUG_STREAM("Filling output ports with message: " << (prefix == "" ? name : prefix + "." + name));

      fillOutputPortsWithMessage(compound[name], prefix == "" ? name : prefix + "." + name);
    }
  }
  else if (message.type() == ros_babel_fish::MessageTypes::Array)
  {
    auto& base = message.as<ros_babel_fish::ArrayMessageBase>();
    switch (base.elementType())
    {
      case ros_babel_fish::MessageTypes::None:
        break;
      case ros_babel_fish::MessageTypes::Bool: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<bool>>();
        std::vector<bool> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<bool>>(prefix, temp_value);
      }
      break;
      case ros_babel_fish::MessageTypes::UInt8: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<uint8_t>>();
        std::vector<uint8_t> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<uint8_t>>(prefix, temp_value);
      }
      break;
      case ros_babel_fish::MessageTypes::UInt16: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<uint16_t>>();
        std::vector<uint16_t> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<uint16_t>>(prefix, temp_value);
      }
      break;
      case ros_babel_fish::MessageTypes::UInt32: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<uint32_t>>();
        std::vector<uint32_t> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<uint32_t>>(prefix, temp_value);
      }
      break;
      case ros_babel_fish::MessageTypes::UInt64: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<uint64_t>>();
        std::vector<uint64_t> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<uint64_t>>(prefix, temp_value);
      }
      break;

      case ros_babel_fish::MessageTypes::Int8: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<int8_t>>();
        std::vector<int8_t> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<int8_t>>(prefix, temp_value);
      }
      break;
      case ros_babel_fish::MessageTypes::Int16: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<int16_t>>();
        std::vector<int16_t> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<int16_t>>(prefix, temp_value);
      }
      break;
      case ros_babel_fish::MessageTypes::Int32: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<int32_t>>();
        std::vector<int32_t> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<int32_t>>(prefix, temp_value);
      }
      break;
      case ros_babel_fish::MessageTypes::Int64: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<int64_t>>();
        std::vector<int64_t> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<int64_t>>(prefix, temp_value);
      }
      break;
      case ros_babel_fish::MessageTypes::Float32: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<float>>();
        std::vector<float> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<float>>(prefix, temp_value);
      }
      break;
      case ros_babel_fish::MessageTypes::Float64: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<double>>();
        std::vector<double> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<double>>(prefix, temp_value);
      }
      break;
      case ros_babel_fish::MessageTypes::Time:
        ROS_ERROR_STREAM("Filling Time Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::Duration:
        ROS_ERROR_STREAM("Filling Duration Array field NOT implemented yet");
        break;
      case ros_babel_fish::MessageTypes::String: {
        auto& massage = base.as<ros_babel_fish::ArrayMessage<std::string>>();
        std::vector<std::string> temp_value;
        for (auto i = 0; i < massage.length(); i++)
        {
          temp_value.push_back(massage[i]);
          ROS_DEBUG_STREAM(prefix << "[" << i << "]"
                                  << " : " << massage[i]);
        }
        tree_node_.setOutput<std::vector<std::string>>(prefix, temp_value);
      }
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
    switch (message.type())
    {
      case ros_babel_fish::MessageTypes::Array:
      case ros_babel_fish::MessageTypes::Compound:
      case ros_babel_fish::MessageTypes::None:
        ROS_DEBUG_STREAM(prefix << " is Array/Compound/None");
        break;
      case ros_babel_fish::MessageTypes::Bool:
        tree_node_.setOutput<bool>(prefix, message.value<bool>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<bool>());
        break;
      case ros_babel_fish::MessageTypes::UInt8:
        tree_node_.setOutput<uint8_t>(prefix, message.value<uint8_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<uint8_t>());
        break;
      case ros_babel_fish::MessageTypes::UInt16:
        tree_node_.setOutput<uint16_t>(prefix, message.value<uint16_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<uint16_t>());
        break;
      case ros_babel_fish::MessageTypes::UInt32:
        tree_node_.setOutput<uint32_t>(prefix, message.value<uint32_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<uint32_t>());
        break;
      case ros_babel_fish::MessageTypes::UInt64:
        tree_node_.setOutput<uint64_t>(prefix, message.value<uint64_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<uint64_t>());
        break;
      case ros_babel_fish::MessageTypes::Int8:
        tree_node_.setOutput<int8_t>(prefix, message.value<int8_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<int8_t>());
        break;
      case ros_babel_fish::MessageTypes::Int16:
        tree_node_.setOutput<int16_t>(prefix, message.value<int16_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<int16_t>());
        break;
      case ros_babel_fish::MessageTypes::Int32:
        tree_node_.setOutput<int32_t>(prefix, message.value<int32_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<int32_t>());
        break;
      case ros_babel_fish::MessageTypes::Int64:
        tree_node_.setOutput<int64_t>(prefix, message.value<int64_t>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<int64_t>());
        break;
      case ros_babel_fish::MessageTypes::Float32:
        tree_node_.setOutput<float>(prefix, message.value<float>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<float>());
        break;
      case ros_babel_fish::MessageTypes::Float64:
        tree_node_.setOutput<double>(prefix, message.value<double>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<double>());
        break;
      case ros_babel_fish::MessageTypes::Time:
        tree_node_.setOutput<ros::Time>(prefix, message.value<ros::Time>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<ros::Time>());
        break;
      case ros_babel_fish::MessageTypes::Duration:
        tree_node_.setOutput<ros::Duration>(prefix, message.value<ros::Duration>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<ros::Duration>());
        break;
      case ros_babel_fish::MessageTypes::String:
        tree_node_.setOutput<std::string>(prefix, message.value<std::string>());
        ROS_DEBUG_STREAM(prefix << " : " << message.value<std::string>());
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
        tockens.push_back(tocken);
      }
    }
  }

  return tockens;
}
template <>
uint8_t convertFromString<uint8_t>(StringView str)
{
  uint8_t result = 0;
  auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
  if (ec != std::errc())
  {
    throw RuntimeError(StrCat("Can't convert string [", str, "] to uint8_t"));
  }
  return result;
}
template <>
uint16_t convertFromString<uint16_t>(StringView str)
{
  uint16_t result = 0;
  auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
  if (ec != std::errc())
  {
    throw RuntimeError(StrCat("Can't convert string [", str, "] to uint16_t"));
  }
  return result;
}
template <>
uint32_t convertFromString<uint32_t>(StringView str)
{
  uint32_t result = 0;
  auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
  if (ec != std::errc())
  {
    throw RuntimeError(StrCat("Can't convert string [", str, "] to uint32_t"));
  }
  return result;
}
template <>
uint64_t convertFromString<uint64_t>(StringView str)
{
  uint64_t result = 0;
  auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
  if (ec != std::errc())
  {
    throw RuntimeError(StrCat("Can't convert string [", str, "] to uint64_t"));
  }
  return result;
}
template <>
int8_t convertFromString<int8_t>(StringView str)
{
  int8_t result = 0;
  auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
  if (ec != std::errc())
  {
    throw RuntimeError(StrCat("Can't convert string [", str, "] to int8_t"));
  }
  return result;
}
template <>
int16_t convertFromString<int16_t>(StringView str)
{
  int16_t result = 0;
  auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
  if (ec != std::errc())
  {
    throw RuntimeError(StrCat("Can't convert string [", str, "] to int16_t"));
  }
  return result;
}
template <>
int32_t convertFromString<int32_t>(StringView str)
{
  int32_t result = 0;
  auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
  if (ec != std::errc())
  {
    throw RuntimeError(StrCat("Can't convert string [", str, "] to int32_t"));
  }
  return result;
}
template <>
int64_t convertFromString<int64_t>(StringView str)
{
  int64_t result = 0;
  auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
  if (ec != std::errc())
  {
    throw RuntimeError(StrCat("Can't convert string [", str, "] to int64_t"));
  }
  return result;
}
}  // end namespace BT
