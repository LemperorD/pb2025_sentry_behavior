// Copyright 2025
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include "pb2025_sentry_behavior/plugins/condition/is_combat_cooldown_ready.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

IsCombatCooldownReady::IsCombatCooldownReady(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsCombatCooldownReady::checkReady, this), config)
{
}

BT::NodeStatus IsCombatCooldownReady::checkReady()
{
  double ready_time = 0.0;
  if (!getInput("ready_time", ready_time)) {
    RCLCPP_WARN(
      rclcpp::get_logger("IsCombatCooldownReady"),
      "[%s] ready_time input missing, assuming cooldown not ready", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double now_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();

  return (now_sec >= ready_time) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsCombatCooldownReady::providedPorts()
{
  return {BT::InputPort<double>(
    "ready_time", 0.0,
    "Monotonic timestamp (seconds) when combat cooldown expires; SUCCESS once now >= ready_time")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsCombatCooldownReady>("IsCombatCooldownReady");
}
