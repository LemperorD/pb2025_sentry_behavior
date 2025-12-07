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

#include "pb2025_sentry_behavior/plugins/subscriber/sub_mode_subscriber.hpp"

#include "behaviortree_ros2/plugins.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

SubModeSubscriber::SubModeSubscriber(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<std_msgs::msg::Bool>(name, config, params)
{
}

BT::PortsList SubModeSubscriber::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::OutputPort<bool>(
      "sub_mode",
      "Latched sub-mode flag mirrored from the /sub_mode topic"),
  };
  return providedBasicPorts(additional_ports);
}

BT::NodeStatus SubModeSubscriber::onTick(const std::shared_ptr<std_msgs::msg::Bool> & last_msg)
{
  if (last_msg) {
    has_state_ = true;
    last_value_ = last_msg->data;
    setOutput("sub_mode", last_value_);
    RCLCPP_DEBUG(logger(), "SubModeSubscriber received sub_mode=%s", last_value_ ? "true" : "false");
  } else if (has_state_) {
    setOutput("sub_mode", last_value_);
  }

  return has_state_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

bool SubModeSubscriber::latchLastMessage() const
{
  return true;
}

}  // namespace pb2025_sentry_behavior

CreateRosNodePlugin(pb2025_sentry_behavior::SubModeSubscriber, "SubModeSubscriber");
