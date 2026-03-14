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

#include "pb2025_sentry_behavior/pb2025_sentry_behavior_server.hpp"

#include <filesystem>
#include <fstream>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "behaviortree_cpp/xml_parsing.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
namespace pb2025_sentry_behavior
{

namespace
{

std::vector<std::string> splitBySemicolon(const std::string & text)
{
  std::vector<std::string> parts;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ';')) {
    parts.push_back(item);
  }
  return parts;
}

geometry_msgs::msg::PoseStamped parseGoalParam(const std::string & goal_str)
{
  auto parts = splitBySemicolon(goal_str);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";

  if (parts.size() == 3) {
    goal.pose.position.x = std::stod(parts[0]);
    goal.pose.position.y = std::stod(parts[1]);
    goal.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::stod(parts[2]));
    goal.pose.orientation = tf2::toMsg(q);
    return goal;
  }

  if (parts.size() == 7) {
    goal.pose.position.x = std::stod(parts[0]);
    goal.pose.position.y = std::stod(parts[1]);
    goal.pose.position.z = std::stod(parts[2]);
    goal.pose.orientation.x = std::stod(parts[3]);
    goal.pose.orientation.y = std::stod(parts[4]);
    goal.pose.orientation.z = std::stod(parts[5]);
    goal.pose.orientation.w = std::stod(parts[6]);
    return goal;
  }

  throw std::invalid_argument(
          "Goal format must be 'x;y;yaw' or 'x;y;z;qx;qy;qz;qw', got: " + goal_str);
}

}  // namespace

template <typename T>
void SentryBehaviorServer::subscribe(
  const std::string & topic, const std::string & bb_key, const rclcpp::QoS & qos)
{
  auto sub = node()->create_subscription<T>(
    topic, qos,
    [this, bb_key](const typename T::SharedPtr msg) { globalBlackboard()->set(bb_key, *msg); });
  subscriptions_.push_back(sub);
}

SentryBehaviorServer::SentryBehaviorServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options)
{
  node()->declare_parameter("use_cout_logger", false);
  node()->get_parameter("use_cout_logger", use_cout_logger_);

  node()->declare_parameter("mode1_goal", "4.65;-3.5;0");
  node()->declare_parameter("mode3_goal", "0;0;0");

  std::string mode1_goal_param;
  std::string mode3_goal_param;
  node()->get_parameter("mode1_goal", mode1_goal_param);
  node()->get_parameter("mode3_goal", mode3_goal_param);

  try {
    globalBlackboard()->set("mode1_goal", parseGoalParam(mode1_goal_param));
    globalBlackboard()->set("mode3_goal", parseGoalParam(mode3_goal_param));
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node()->get_logger(),
      "Invalid mode goal parameter (%s). Falling back to defaults.",
      e.what());
    globalBlackboard()->set("mode1_goal", parseGoalParam("4.65;-3.5;0"));
    globalBlackboard()->set("mode3_goal", parseGoalParam("0;0;0"));
  }

  subscribe<pb_rm_interfaces::msg::EventData>("referee/event_data", "referee_eventData");
  subscribe<pb_rm_interfaces::msg::GameRobotHP>("/referee/all_robot_hp", "referee_allRobotHP");
  subscribe<pb_rm_interfaces::msg::GameStatus>("/referee/game_status", "referee_gameStatus");
  subscribe<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", "referee_groundRobotPosition");
  subscribe<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", "referee_rfidStatus");
  subscribe<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", "referee_robotStatus");
  subscribe<pb_rm_interfaces::msg::Buff>("referee/buff", "referee_buff");

  auto tracker_qos = rclcpp::SensorDataQoS();
  {
    geometry_msgs::msg::Point init_enemy_pos;
    init_enemy_pos.x = 0.0;
    init_enemy_pos.y = 0.0;
    init_enemy_pos.z = 0.0;
    globalBlackboard()->set("serial_enemyPos", init_enemy_pos);
    globalBlackboard()->set("serial_enemyPosStamp", -1.0);

    auto enemy_pos_sub = node()->create_subscription<geometry_msgs::msg::Point>(
      "/serial/EnemyPos", tracker_qos,
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        globalBlackboard()->set("serial_enemyPos", *msg);
        const auto now = std::chrono::steady_clock::now();
        const double now_sec = std::chrono::duration<double>(now.time_since_epoch()).count();
        globalBlackboard()->set("serial_enemyPosStamp", now_sec);
      });
    subscriptions_.push_back(enemy_pos_sub);
  }

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  subscribe<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", "nav_globalCostmap", costmap_qos);
}

bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)
{
  RCLCPP_INFO(
    node()->get_logger(), "onGoalReceived with tree name '%s' with payload '%s'", tree_name.c_str(),
    payload.c_str());
  return true;
}

void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;
}

std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  return std::nullopt;
}

std::optional<std::string> SentryBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  RCLCPP_INFO(
    node()->get_logger(), "onTreeExecutionCompleted with status=%d (canceled=%d) after %d ticks",
    static_cast<int>(status), was_cancelled, tick_count_);
  logger_cout_.reset();
  std::string result = treeName() +
                       " tree completed with status=" + std::to_string(static_cast<int>(status)) +
                       " after " + std::to_string(tick_count_) + " ticks";
  return result;
}

}  // namespace pb2025_sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<pb2025_sentry_behavior::SentryBehaviorServer>(options);

  RCLCPP_INFO(action_server->node()->get_logger(), "Starting SentryBehaviorServer");

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  // Groot2 editor requires a model of your registered Nodes.
  // You don't need to write that by hand, it can be automatically
  // generated using the following command.
  std::string xml_models = BT::writeTreeNodesModelXML(action_server->factory());

  // Save the XML models to a file
  std::ofstream file(std::filesystem::path(ROOT_DIR) / "behavior_trees" / "models.xml");
  file << xml_models;

  rclcpp::shutdown();
}
