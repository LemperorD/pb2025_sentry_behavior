#include "pb2025_sentry_behavior/plugins/action/handle_path_blocked.hpp"

namespace pb2025_sentry_behavior
{

HandlePathBlockedAction::HandlePathBlockedAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus HandlePathBlockedAction::tick()
{
  // Placeholder: real logic could publish a request to clear the path, switch behavior, etc.
  std::string info;
  if (getInput("key_port", info)) {
    RCLCPP_INFO(logger_, "HandlePathBlocked: nav_path_status=%s", info.c_str());
  } else {
    RCLCPP_INFO(logger_, "HandlePathBlocked: nav_path_status not set");
  }

  // For now just succeed quickly so tree can continue; change to RUNNING if action is long-lived
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList HandlePathBlockedAction::providedPorts()
{
  return {BT::InputPort<std::string>("key_port", "{@nav_path_status}", "Navigation path status key")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::HandlePathBlockedAction>("HandlePathBlocked");
}
