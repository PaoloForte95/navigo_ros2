// Copyright (c) 2022 Paolo Forte
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

#include <vector>
#include <string>

#include "orunav2_behavior_tree/plugins/condition/planner_updated_condition.hpp"

namespace orunav2_behavior_tree
{

PlannerUpdatedCondition::PlannerUpdatedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  first_time(true)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus PlannerUpdatedCondition::tick()
{
  if (first_time) {
    first_time = false;
    config().blackboard->get<std::string>("planner", planner_);
    std::cout << "Setting first planner:" << planner_ << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  std::string current_planner;
  config().blackboard->get<std::string>("planner", current_planner);
  
  std::cout << "Current global planner: "<< current_planner << " first planner: " << planner_ << std::endl;
  if (planner_ != current_planner ) {
    std::cout << "Changed planner from: "<< planner_ << " to: " << current_planner << std::endl;
    planner_ = current_planner;
    return BT::NodeStatus::SUCCESS;
  }
  

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orunav2_behavior_tree::PlannerUpdatedCondition>("UpdatedPlanner");
}
