#include "behavior_tree_plugin_ros/ros_wrapper.h"

#include "ros/callback_queue.h"

namespace behavior_tree_plugin_ros
{
RosWrapper::RosWrapper(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle& node_handle)
  : BT::ControlNode(name, config), node_handle_(node_handle)

{
  ROS_INFO_STREAM("RosWrapper for : " << this->name());

}
BT::PortsList RosWrapper::providedPorts()
{
  return {
    BT::InputPort<bool>("run_always"),
    };
}

std::string RosWrapper::getStatusTopicName()
{
  return "status";
}
std::string RosWrapper::getCommandTopicName()
{
  return "tick";
}

int RosWrapper::getIdleCode()
{
  return 0;
}
int RosWrapper::getRunningCode()
{
  return 1;
}
int RosWrapper::getSuccessCode()
{
  return 2;
}
int RosWrapper::getFailureCode()
{
  return 3;
}

void RosWrapper::OnChildSuccess(int i)
{
  children_status[i].data=getSuccessCode();
  if(children_command_.size()>0)
  {
    children_command_[i]=-1;
  }
}
void RosWrapper::OnChildFailure(int i)
{
  children_status[i].data=getFailureCode();
  if(children_command_.size()>0)
  {
    children_command_[i]=-1;
  }
}
void RosWrapper::OnChildRunning(int i)
{
  children_status[i].data=getRunningCode();
}
void RosWrapper::OnChildStatusInitialize(int i)
{
  BT::TreeNode* child_node = children_nodes_[i];
  std::string topic_name_status=child_node->name()+"/"+getStatusTopicName();
  
  auto status_publisher = node_handle_.advertise<std_msgs::Int16>(topic_name_status, 1);
  children_status_publisher_.push_back(status_publisher);
  
  std_msgs::Int16 status;
  status.data=getIdleCode();
  children_status.push_back(status);
}
void RosWrapper::OnChildCommandInitialize(int i)
{
  BT::TreeNode* child_node = children_nodes_[i];
  std::string topic_name_command=child_node->name()+"/"+getCommandTopicName();

  
  auto command_call_back=[&](const std_msgs::BoolConstPtr& msg,const int& i,std::string & topic_name) {
    children_command_[i] = msg->data;
    ROS_INFO_STREAM("callback received:"<<topic_name );
  };
  children_command_call_back_.push_back(command_call_back);

  auto command_subscriber=node_handle_.subscribe<std_msgs::Bool>(topic_name_command, 1, boost::bind(children_command_call_back_[i], _1, i,topic_name_command) );
  children_command_subscriber_.push_back(command_subscriber);
  
  int command=-1;
  children_command_.push_back(command); 
  
}

bool RosWrapper::isRunAlwaysActive()
{
  BT::Optional<bool> run_always = getInput<bool>("run_always");
  if(run_always)
  {
     return run_always.value();
  }
  return false;

}

void RosWrapper::initialize()
{
  const size_t children_count = children_nodes_.size();

  for (unsigned int i = 0; i < children_count; i++)
  {
     OnChildStatusInitialize(i);
    if(getCommandTopicName()!="")
    {
      OnChildCommandInitialize(i);
    }

    // BT::PortsList child_ports_=child_node_.getPorts();
    //   for(auto port_name_info:child_ports)
    // {
    //   auto port_name=port_name_info.first();
    //   auto port_info=port_name_info.second();
    //   if(port_info.direction()==PortDirection.INPUT)
    //   {
    //     //subscribe to a topic with a callback
    //   }else if(port_info.direction()==PortDirection.OUTPUT)
    //   {

    //     //register a publisher
    //   }

    // }
    }
}

BT::NodeStatus RosWrapper::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
  const size_t children_count = children_nodes_.size();

  for (unsigned int i = 0; i < children_count; i++)
  {
    BT::TreeNode* child_node = children_nodes_[i];
  
  //set input ports based on the subscribed topics
  // for(auto port_name_info:child_node_.config().input_ports)
  // {

  //   auto port_name=port_name_info.first();
  //   auto port_info=port_name_info.second();
  //   child_node_.config().blackboard.set(port_name,value);

  // }
  if(isRunAlwaysActive() || (children_command_.size()>0 &&children_command_[i]==1))
  {
    const BT::NodeStatus child_state = child_node->executeTick();
     switch (child_state)
      {
          case BT::NodeStatus::SUCCESS:
            OnChildSuccess(i);
            break;

          case BT::NodeStatus::FAILURE:
            OnChildFailure(i);
            break;
          case BT::NodeStatus::RUNNING:
            OnChildRunning(i);
            break;

          default:
            throw BT::LogicError("A child node must never return IDLE");
      }
        
    }else if(children_command_.size()>0 && children_command_[i]==0 && children_status[i].data==getRunningCode())
    {
      haltChild(i);
      children_status[i].data=getIdleCode();
      
    }
    children_status_publisher_[i].publish(children_status[i]);
     // publish topics based on output ports
    // for(auto port_name_info:child_node_.config().output_ports)
    // {
    //   auto port_name=port_name_info.first();
    //   auto port_info=port_name_info.second();
    //   child_node_.config().blackboard.get(port_name,value);
    //   publisher.publish(value);
    // }
     
  }
  
  
 
  
  
  return BT::NodeStatus::RUNNING;
}
void RosWrapper::halt()
{
    haltChildren();
    BT::ControlNode::halt();
}
/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
void RosWrapper::Register(BT::BehaviorTreeFactory& factory, const std::string& registration_ID,
                              ros::NodeHandle& node_handle)
{
  BT::NodeBuilder builder = [&node_handle](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<RosWrapper>(name, config, node_handle);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<RosWrapper>();
  manifest.ports = RosWrapper::providedPorts();
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}
}  // end namespace BT
