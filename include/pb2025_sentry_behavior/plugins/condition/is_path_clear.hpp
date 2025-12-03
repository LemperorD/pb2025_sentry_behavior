// Copyright 2025
// Licensed under the Apache License, Version 2.0

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_CLEAR_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_CLEAR_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsPathClearCondition : public BT::SimpleConditionNode
{
public:
  IsPathClearCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkClear();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsPathClearCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_CLEAR_HPP_
