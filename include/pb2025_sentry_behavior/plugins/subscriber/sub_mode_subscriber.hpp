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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__SUBSCRIBER__SUB_MODE_SUBSCRIBER_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__SUBSCRIBER__SUB_MODE_SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "std_msgs/msg/bool.hpp"

namespace pb2025_sentry_behavior
{

class SubModeSubscriber : public BT::RosTopicSubNode<std_msgs::msg::Bool>
{
public:
  SubModeSubscriber(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool> & last_msg) override;

  bool latchLastMessage() const override;

private:
  bool has_state_ = false;
  bool last_value_ = false;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__SUBSCRIBER__SUB_MODE_SUBSCRIBER_HPP_
