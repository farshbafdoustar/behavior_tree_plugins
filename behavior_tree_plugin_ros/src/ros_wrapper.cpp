#include "behavior_tree_plugin_ros/ros_wrapper.h"

#include "ros/callback_queue.h"

namespace behavior_tree_plugin_ros
{
RosWrapper::RosWrapper(const std::string& name, const BT::NodeConfig& config, ros::NodeHandle& node_handle)
  : BT::ControlNode(name, config), node_handle_(node_handle)

{
  ROS_INFO_STREAM("RosWrapper for : " << this->name());
}
BT::PortsList RosWrapper::providedPorts()
{
  return {
    BT::InputPort<bool>("run_always"),
    BT::InputPort<double>("frequency"),
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

void RosWrapper::UpdateAndPublishChildrenStatus(int i, int status)
{
  if (children_status_[i].data != status)
  {
    children_status_[i].data = status;
    children_status_publisher_[i].publish(children_status_[i]);
  }
}
void RosWrapper::OnHalt()
{
}
void RosWrapper::OnChildSuccess(int i)
{
  UpdateAndPublishChildrenStatus(i, getSuccessCode());
  if (children_command_.size() > 0)
  {
    children_command_[i] = -1;
  }
}
void RosWrapper::OnChildFailure(int i)
{
  UpdateAndPublishChildrenStatus(i, getFailureCode());

  if (children_command_.size() > 0)
  {
    children_command_[i] = -1;
  }
}
void RosWrapper::OnChildRunning(int i)
{
  UpdateAndPublishChildrenStatus(i, getRunningCode());
}
void RosWrapper::OnChildStatusInitialize(int i)
{
  BT::TreeNode* child_node = children_nodes_[i];
  std::string topic_name_status = child_node->name() + "/" + getStatusTopicName();

  auto status_publisher = node_handle_.advertise<std_msgs::Int16>(topic_name_status, 1, true);
  children_status_publisher_.push_back(status_publisher);

  std_msgs::Int16 status;
  status.data = getIdleCode();
  children_status_.push_back(status);
}
void RosWrapper::OnChildCommandInitialize(int i)
{
  BT::TreeNode* child_node = children_nodes_[i];
  std::string topic_name_command = child_node->name() + "/" + getCommandTopicName();

  auto command_call_back = [&](const std_msgs::BoolConstPtr& msg, const int& i, std::string& topic_name) {
    children_command_[i] = msg->data;
    ROS_INFO_STREAM("callback received:" << topic_name);
  };
  children_command_call_back_.push_back(command_call_back);

  auto command_subscriber = node_handle_.subscribe<std_msgs::Bool>(
      topic_name_command, 1, boost::bind(children_command_call_back_[i], _1, i, topic_name_command));
  children_command_subscriber_.push_back(command_subscriber);

  int command = -1;
  children_command_.push_back(command);
}

bool RosWrapper::isRunAlwaysActive()
{
  BT::Expected<bool> run_always = getInput<bool>("run_always");
  if (run_always)
  {
    return run_always.value();
  }
  return false;
}

void RosWrapper::onNewState(const std_msgs::BoolConstPtr& msg)
{
  ROS_INFO_STREAM("Halt Receieved for : " << this->name());
  if (msg)
  {
    ROS_INFO_STREAM("Receieved  " << msg->data << " for : " << this->name());
    if (msg->data)
    {
      ROS_INFO_STREAM("Halting Children Started for  : " << this->name());
      OnHalt();
      haltChildren();
    }
  }

  ROS_INFO_STREAM("Halting Children Finished for  : " << this->name());
}
void RosWrapper::initialize()
{
  ros::Time::init();
  ros::Time::waitForValid();
  last_tick_time_ = ros::Time::now();

  BT::Expected<double> frequency = getInput<double>("frequency");
  if (frequency)
  {
    frequency_ = frequency.value();
  }
  ROS_INFO_STREAM("RosWrapper initialized with frequency: " << frequency_);

  halt_subscriber_ = node_handle_.subscribe("halt", 1, &RosWrapper::onNewState, this);

  const size_t children_count = children_nodes_.size();

  for (unsigned int i = 0; i < children_count; i++)
  {
    OnChildStatusInitialize(i);
    if (getCommandTopicName() != "")
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
  // in simulation it could take time until getting correct time
  if (last_tick_time_ > ros::Time::now())
  {
    last_tick_time_ = ros::Time::now();
  }
  ROS_DEBUG_STREAM("last_tick_time_  " << last_tick_time_);
  ROS_DEBUG_STREAM("ros::Time::now()  " << ros::Time::now());
  if (frequency_ > 0.0 && last_tick_time_ + ros::Duration(1.0 / frequency_) < ros::Time::now())
  {
    last_tick_time_ = ros::Time::now();
    ROS_DEBUG_STREAM("tick  ");
  }
  else if (frequency_ == 0.0)
  {
    ROS_DEBUG_STREAM("frequency_==0  ");
  }
  else
  {
    ROS_DEBUG_STREAM("Nothing");
    setStatus(BT::NodeStatus::RUNNING);
    return BT::NodeStatus::RUNNING;
  }
  setStatus(BT::NodeStatus::RUNNING);

  // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
  const size_t children_count = children_nodes_.size();

  for (unsigned int i = 0; i < children_count; i++)
  {
    BT::TreeNode* child_node = children_nodes_[i];

    // set input ports based on the subscribed topics
    //  for(auto port_name_info:child_node_.config().input_ports)
    //  {

    //   auto port_name=port_name_info.first();
    //   auto port_info=port_name_info.second();
    //   child_node_.config().blackboard.set(port_name,value);

    // }
    if (isRunAlwaysActive() || (children_command_.size() > 0 && children_command_[i] == 1))
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
          // only chnage it for non indicators
          if (!isRunAlwaysActive())
          {
            OnChildRunning(i);
          }
          break;

        default:
          throw BT::LogicError("A child node must never return IDLE");
      }
    }
    else if (children_command_.size() > 0 && children_command_[i] == 0 && children_status_[i].data == getRunningCode())
    {
      haltChild(i);
      UpdateAndPublishChildrenStatus(i, getIdleCode());
    }
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
  BT::NodeBuilder builder = [&node_handle](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<RosWrapper>(name, config, node_handle);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<RosWrapper>();
  manifest.ports = RosWrapper::providedPorts();
  manifest.registration_ID = registration_ID;
  factory.registerBuilder(manifest, builder);
}
}  // namespace behavior_tree_plugin_ros
