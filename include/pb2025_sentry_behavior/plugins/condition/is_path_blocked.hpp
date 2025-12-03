// Copyright 2025
// Licensed under the Apache License, Version 2.0

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_BLOCKED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_BLOCKED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsPathBlockedCondition : public BT::SimpleConditionNode
{
public:
  IsPathBlockedCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkBlocked();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsPathBlockedCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_BLOCKED_HPP_
