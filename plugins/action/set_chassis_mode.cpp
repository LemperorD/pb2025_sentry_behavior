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

#include "pb2025_sentry_behavior/plugins/action/set_chassis_mode.hpp"

namespace pb2025_sentry_behavior
{

SetChassisModeAction::SetChassisModeAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList SetChassisModeAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<int>("mode", 1, "Chassis Mode: 1=Follow, 2=Spin, 3=Survival"),
    BT::InputPort<int>(
      "current_mode_in", -1,
      "Current chassis mode read from the blackboard (map to {@last_mode}) to avoid redundant writes"),
    BT::OutputPort<int>("current_mode", "Current chassis mode written when changed"),
    BT::OutputPort<double>(
      "combat_cooldown_ready_time",
      "Monotonic timestamp (seconds) indicating when switching from mode 2 to mode 1 is allowed"),
  };
  return providedBasicPorts(additional_ports);
}

bool SetChassisModeAction::setMessage(std_msgs::msg::Int32 & msg)
{
  int mode = 1;
  getInput("mode", mode);

  msg.data = mode;

  // write the new mode back to the blackboard so other nodes can read it
  int prev_mode = -1;
  if (getInput("current_mode_in", prev_mode) && prev_mode == mode) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "SetChassisMode: mode %d unchanged, skipping blackboard write", mode);
  } else {
    setOutput("current_mode", mode);
  }

  const double now_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  if (mode == 2) {
    setOutput("combat_cooldown_ready_time", now_sec + 3.0);
  } else {
    setOutput("combat_cooldown_ready_time", now_sec);
  }

  RCLCPP_INFO(node_->get_logger(), "SetChassisMode publishing mode=%d", mode);
  return true;
}

bool SetChassisModeAction::setHaltMessage(std_msgs::msg::Int32 & msg)
{
  // On halt, we publish 0 to indicate stop / no mode (optional)
  msg.data = 0;
  setOutput("current_mode", 0);
  setOutput("combat_cooldown_ready_time", std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::SetChassisModeAction, "SetChassisMode");
