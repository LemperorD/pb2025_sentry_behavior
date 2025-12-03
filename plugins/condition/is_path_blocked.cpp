#include "pb2025_sentry_behavior/plugins/condition/is_path_blocked.hpp"

namespace pb2025_sentry_behavior
{

IsPathBlockedCondition::IsPathBlockedCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsPathBlockedCondition::checkBlocked, this), config)
{
}

BT::NodeStatus IsPathBlockedCondition::checkBlocked()
{
  // Try to read a boolean flag from blackboard indicating blockage
  auto maybe_bool = getInput<bool>("key_port");
  if (maybe_bool) {
    return (*maybe_bool) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // Fallback to integer interpretation (non-zero = blocked)
  auto maybe_int = getInput<int>("key_port");
  if (maybe_int) {
    return ((*maybe_int) != 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  RCLCPP_WARN(logger_.get_child("is_path_blocked"), "nav_path_status not found or unsupported type on blackboard");
  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsPathBlockedCondition::providedPorts()
{
  return {BT::InputPort<bool>("key_port", false, "Navigation path blocked flag (bool) or int)")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsPathBlockedCondition>("IsPathBlocked");
}
