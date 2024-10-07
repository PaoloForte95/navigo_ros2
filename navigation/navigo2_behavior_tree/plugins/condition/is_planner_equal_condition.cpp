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

#include "navigo2_behavior_tree/plugins/condition/is_planner_equal_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace navigo2{
namespace behavior_tree{

IsPlannerEqualCondition::IsPlannerEqualCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput<std::string>("planner_to_check", planner_id_);
 
}

BT::NodeStatus IsPlannerEqualCondition::tick()
{
  config().blackboard->get<std::string>("planner", actual_planner_);
  RCLCPP_INFO(node_->get_logger(), "Planner to check: %s and actual planner: %s", planner_id_.c_str(), actual_planner_.c_str());
  if(actual_planner_ == "" || actual_planner_ == "NotComputed" || planner_id_ != actual_planner_){
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
  
}

}
}  

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo2::behavior_tree::IsPlannerEqualCondition>("IsPlannerEqual");
}
