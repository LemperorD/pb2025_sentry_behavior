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

#include "pb2025_sentry_behavior/plugins/condition/is_game_status.hpp"

namespace pb2025_sentry_behavior
{

IsGameStatusCondition::IsGameStatusCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGameStatusCondition::checkGameStart, this), config)
{
}

BT::NodeStatus IsGameStatusCondition::checkGameStart()
{
  int expected_game_progress, min_remain_time, max_remain_time;
  double confirm_duration_sec = 0.0;
  auto msg = getInput<pb_rm_interfaces::msg::GameStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "GameStatus message is not available");
    match_start_time_initialized_ = false;
    return BT::NodeStatus::FAILURE;
  }

  getInput("expected_game_progress", expected_game_progress);
  getInput("min_remain_time", min_remain_time);
  getInput("max_remain_time", max_remain_time);
  getInput("confirm_duration_sec", confirm_duration_sec);

  RCLCPP_DEBUG(
    logger_, "Checking: Progress(%d/%d), Remain Time(%ds) in [%d-%d]",
    static_cast<int>(msg->game_progress), expected_game_progress, msg->stage_remain_time,
    min_remain_time, max_remain_time);

  const bool is_progress_match = (msg->game_progress == expected_game_progress);
  const bool is_time_in_range =
    (msg->stage_remain_time >= min_remain_time) && (msg->stage_remain_time <= max_remain_time);

  const bool is_match = is_progress_match && is_time_in_range;
  if (!is_match) {
    match_start_time_initialized_ = false;
    return BT::NodeStatus::FAILURE;
  }

  if (confirm_duration_sec <= 0.0) {
    return BT::NodeStatus::SUCCESS;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!match_start_time_initialized_) {
    match_start_time_initialized_ = true;
    match_start_time_ = now;
    RCLCPP_DEBUG(logger_, "GameStatus matched, start confirm timer: %.2fs", confirm_duration_sec);
    return BT::NodeStatus::FAILURE;
  }

  const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - match_start_time_);
  if (elapsed.count() >= confirm_duration_sec) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsGameStatusCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameStatus>(
      "key_port", "{@referee_gameStatus}", "GameStatus port on blackboard"),
    BT::InputPort<int>("expected_game_progress", 4, "Expected game progress stage"),
    BT::InputPort<int>("min_remain_time", 0, "Minimum remaining time (s)"),
    BT::InputPort<int>("max_remain_time", 420, "Maximum remaining time (s)"),
    BT::InputPort<double>(
      "confirm_duration_sec", 0.0,
      "Required continuous match duration (s) before returning SUCCESS"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsGameStatusCondition>("IsGameStatus");
}
