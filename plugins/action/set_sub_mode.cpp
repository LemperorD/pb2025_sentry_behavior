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

#include "pb2025_sentry_behavior/plugins/action/set_sub_mode.hpp"

#include <algorithm>
#include <chrono>

namespace pb2025_sentry_behavior
{

SetSubModeAction::SetSubModeAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList SetSubModeAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<bool>("sub_mode", false, "Sub mode flag: false=go home, true=enemy handling"),
    BT::InputPort<double>(
      "hold_sec", 0.0,
      "When sub_mode=true, minimum hold time in seconds before allowing sub_mode=false"),
    BT::InputPort<bool>(
      "current_sub_mode_in", false,
      "Current sub mode read from blackboard to avoid redundant updates"),
    BT::OutputPort<bool>("current_sub_mode", "Current sub mode written to blackboard"),
    BT::OutputPort<double>(
      "submode_cooldown_ready_time",
      "Monotonic timestamp (seconds) indicating when switching sub_mode from true to false is allowed"),
    BT::OutputPort<double>(
      "submode_hold_until",
      "Monotonic timestamp (seconds) before which sub_mode should remain true"),
  };
  return providedBasicPorts(additional_ports);
}

bool SetSubModeAction::setMessage(std_msgs::msg::Bool & msg)
{
  bool sub_mode = false;
  double hold_sec = 0.0;
  getInput("sub_mode", sub_mode);
  getInput("hold_sec", hold_sec);
  msg.data = sub_mode;

  const double now_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();

  bool prev_sub_mode = false;
  const bool mode_unchanged =
    getInput("current_sub_mode_in", prev_sub_mode) && prev_sub_mode == sub_mode;
  if (mode_unchanged) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "SetSubMode: sub_mode unchanged (%s)", sub_mode ? "true" : "false");
  } else {
    setOutput("current_sub_mode", sub_mode);
  }

  // Match SetChassisMode semantics: refresh cooldown only when mode changes.
  if (!mode_unchanged) {
    if (sub_mode) {
      const double ready_time = now_sec + std::max(0.0, hold_sec);
      setOutput("submode_hold_until", ready_time);
      setOutput("submode_cooldown_ready_time", ready_time);
    } else {
      setOutput("submode_hold_until", now_sec);
      setOutput("submode_cooldown_ready_time", now_sec);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "SetSubMode publishing sub_mode=%s", sub_mode ? "true" : "false");
  return true;
}

bool SetSubModeAction::setHaltMessage(std_msgs::msg::Bool & msg)
{
  msg.data = false;
  setOutput("current_sub_mode", false);
  const double now_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  setOutput("submode_hold_until", now_sec);
  setOutput("submode_cooldown_ready_time", now_sec);
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::SetSubModeAction, "SetSubMode");
