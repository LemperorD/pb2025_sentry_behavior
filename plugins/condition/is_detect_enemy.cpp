// Copyright 2025 Lihan Chen
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

#include "pb2025_sentry_behavior/plugins/condition/is_detect_enemy.hpp"

#include <algorithm>
#include <cmath>

namespace pb2025_sentry_behavior
{

IsDetectEnemyCondition::IsDetectEnemyCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyCondition::checkEnemy, this), config)
{
}

BT::PortsList IsDetectEnemyCondition::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Point>(
      "key_port", "{@serial_enemyPos}", "Enemy position port on blackboard from /serial/EnemyPos"),
    BT::InputPort<double>(
      "stamp_port", "{@serial_enemyPosStamp}",
      "Steady-clock receive timestamp (seconds) for /serial/EnemyPos"),
    BT::InputPort<float>("max_distance", 8.0, "Distance to enemy target"),
    BT::InputPort<double>("fresh_timeout_sec", 0.5, "Freshness timeout in seconds"),
    BT::InputPort<int>("stable_required_count", 3, "Required consecutive valid detections"),
  };
}

BT::NodeStatus IsDetectEnemyCondition::checkEnemy()
{
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  double enemy_pos_stamp_sec;
  float max_distance;
  double fresh_timeout_sec;
  int stable_required_count;

  auto msg = getInput<geometry_msgs::msg::Point>("key_port");
  if (!msg) {
    RCLCPP_INFO_THROTTLE(
      logger_, steady_clock, 5000, "EnemyPos message is not available");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("stamp_port", enemy_pos_stamp_sec)) {
    RCLCPP_INFO_THROTTLE(
      logger_, steady_clock, 5000, "EnemyPos timestamp is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("max_distance", max_distance);
  getInput("fresh_timeout_sec", fresh_timeout_sec);
  getInput("stable_required_count", stable_required_count);

  const bool finite_xy = std::isfinite(msg->x) && std::isfinite(msg->y);
  const float distance_to_enemy = std::hypot(static_cast<float>(msg->x), static_cast<float>(msg->y));
  const bool valid_distance = distance_to_enemy > 1e-3f && distance_to_enemy <= max_distance;
  const auto now = std::chrono::steady_clock::now();
  const double now_sec = std::chrono::duration<double>(now.time_since_epoch()).count();
  const bool fresh_enough = (now_sec - enemy_pos_stamp_sec) <= std::max(0.0, fresh_timeout_sec);
  const bool is_valid_detection = finite_xy && valid_distance && fresh_enough;

  constexpr double kStampEps = 1e-6;
  const bool stamp_moved_backward = enemy_pos_stamp_sec + kStampEps < last_enemy_pos_stamp_sec_;
  const bool is_new_message = enemy_pos_stamp_sec > last_enemy_pos_stamp_sec_ + kStampEps;

  if (stamp_moved_backward) {
    consecutive_valid_count_ = 0;
    last_enemy_pos_stamp_sec_ = enemy_pos_stamp_sec;
  }

  if (!is_valid_detection) {
    consecutive_valid_count_ = 0;
  } else if (is_new_message) {
    consecutive_valid_count_ += 1;
    last_enemy_pos_stamp_sec_ = enemy_pos_stamp_sec;
  }

  const bool stable_enough = consecutive_valid_count_ >= std::max(1, stable_required_count);

  if (fresh_enough && stable_enough && is_valid_detection) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsDetectEnemyCondition>("IsDetectEnemy");
}
