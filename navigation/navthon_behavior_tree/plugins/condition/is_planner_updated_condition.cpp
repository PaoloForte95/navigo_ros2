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

#include "navthon_behavior_tree/plugins/condition/is_planner_updated_condition.hpp"

namespace navthon{
namespace behavior_tree{

PlannerUpdatedCondition::PlannerUpdatedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  previous_planner_ = "";
}

BT::NodeStatus PlannerUpdatedCondition::tick()
{

  std::string current_planner;
  getInput("planner", current_planner);
  if(previous_planner_.empty()){
    config().blackboard->get<std::string>("planner", previous_planner_);
    return BT::NodeStatus::FAILURE;
  }
  
  if (previous_planner_ != current_planner ) {
    RCLCPP_INFO(node_->get_logger(), "Changed planner from %s to %s", previous_planner_.c_str(), current_planner.c_str());
    previous_planner_ = current_planner;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}
}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navthon::behavior_tree::PlannerUpdatedCondition>("IsPlannerUpdated");
}
