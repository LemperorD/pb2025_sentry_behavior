// Copyright 2025
// Licensed under the Apache License, Version 2.0

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__HANDLE_PATH_BLOCKED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__HANDLE_PATH_BLOCKED_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class HandlePathBlockedAction : public BT::SyncActionNode
{
public:
  HandlePathBlockedAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("HandlePathBlockedAction");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__HANDLE_PATH_BLOCKED_HPP_
