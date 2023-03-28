// Copyright (c) 2023 Paolo Forte
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

#include <memory>
#include <string>

#include "orunav2_behavior_tree/plugins/action/global_planner_selector_node.hpp"

namespace orunav2_behavior_tree
{

SelectGlobalPlannerAction::SelectGlobalPlannerAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<orunav2_msgs::action::GlobalPlannerSelector>(xml_tag_name, action_name, conf)
{

  
}

void SelectGlobalPlannerAction::on_tick()
{
  getInput("goal", goal_.goal);
  getInput("selector_id", goal_.selector_id);
  getInput("planner_ids", goal_.planner_ids);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

BT::NodeStatus SelectGlobalPlannerAction::on_success()
{
  setOutput("selected_planner", result_.result->planner_id);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SelectGlobalPlannerAction::on_aborted()
{
  std::string empty_planner;
  setOutput("selected_planner", empty_planner);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SelectGlobalPlannerAction::on_cancelled()
{
  std::string empty_planner;
  setOutput("selected_planner", empty_planner);
  return BT::NodeStatus::SUCCESS;
}

void SelectGlobalPlannerAction::halt()
{
  std::string empty_planner;
  setOutput("selected_planner", empty_planner);
  BtActionNode::halt();
}

}  // namespace orunav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<orunav2_behavior_tree::SelectGlobalPlannerAction>(
        name, "select_global_planner", config);
    };

  factory.registerBuilder<orunav2_behavior_tree::SelectGlobalPlannerAction>(
    "GlobalPlannerSelector", builder);
}
