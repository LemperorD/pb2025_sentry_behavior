#include "pb2025_sentry_behavior/plugins/condition/is_path_clear.hpp"


namespace pb2025_sentry_behavior
{

IsPathClearCondition::IsPathClearCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsPathClearCondition::checkClear, this), config)
{
}

BT::NodeStatus IsPathClearCondition::checkClear()
{
  // If a bool exists and is false -> path clear
  auto maybe_bool = getInput<bool>("key_port");
  if (maybe_bool) {
    return (!(*maybe_bool)) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // Fallback to integer interpretation (zero = clear)
  auto maybe_int = getInput<int>("key_port");
  if (maybe_int) {
    return ((*maybe_int) == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  RCLCPP_WARN(logger_.get_child("is_path_clear"), "nav_path_status not found or unsupported type on blackboard");
  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsPathClearCondition::providedPorts()
{
  return {BT::InputPort<bool>("key_port", false, "Navigation path blocked flag (bool) or int) -- this condition returns SUCCESS when not blocked")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsPathClearCondition>("IsPathClear");
}
