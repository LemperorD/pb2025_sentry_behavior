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
    BT::InputPort<bool>("mode", false, "Sub-mode flag written to the blackboard/topic"),
  };
  return providedBasicPorts(additional_ports);
}

bool SetSubModeAction::setMessage(std_msgs::msg::Bool & msg)
{
  bool mode = false;
  getInput("mode", mode);
  msg.data = mode;
  RCLCPP_INFO(node_->get_logger(), "SetSubMode publishing mode=%s", mode ? "true" : "false");
  return true;
}

bool SetSubModeAction::setHaltMessage(std_msgs::msg::Bool & msg)
{
  msg.data = false;
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::SetSubModeAction, "SetSubMode");
