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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_COMBAT_COOLDOWN_READY_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_COMBAT_COOLDOWN_READY_HPP_

#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace pb2025_sentry_behavior
{

class IsCombatCooldownReady : public BT::SimpleConditionNode
{
public:
  explicit IsCombatCooldownReady(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkReady();

  static BT::PortsList providedPorts();
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_COMBAT_COOLDOWN_READY_HPP_
