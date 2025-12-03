#include "pb2025_sentry_behavior/plugins/action/handle_path_clear.hpp"

namespace pb2025_sentry_behavior
{

HandlePathClearAction::HandlePathClearAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus HandlePathClearAction::tick()
{
  std::string info;
  if (getInput("key_port", info)) {
    RCLCPP_INFO(logger_, "HandlePathClear: nav_path_status=%s", info.c_str());
  } else {
    RCLCPP_INFO(logger_, "HandlePathClear: nav_path_status not set");
  }

  // Placeholder success
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList HandlePathClearAction::providedPorts()
{
  return {BT::InputPort<std::string>("key_port", "{@nav_path_status}", "Navigation path status key")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::HandlePathClearAction>("HandlePathClear");
}
