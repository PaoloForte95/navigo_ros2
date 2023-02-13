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

#include <string>
#include <memory>
#include <limits>



#include "orunav2_behavior_tree/plugins/action/remove_unfeasible_planner_action.hpp"

namespace orunav2_behavior_tree
{

RemoveUnfeasiblePlanner::RemoveUnfeasiblePlanner(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  
}

inline BT::NodeStatus RemoveUnfeasiblePlanner::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  std::string planner_id;
  Planners planner_ids;
  getInput("planner_id", planner_id);
  config().blackboard->get<Planners>("planner_ids", planner_ids);


  if (planner_id.empty()) {
    //Do not remove any planner from the list
    setOutput("planner_ids", planner_ids);
    return BT::NodeStatus::SUCCESS;
  }

  //Find the planner from the current list 
  auto itr = std::find(planner_ids.begin(), planner_ids.end(), planner_id);
  if (itr == planner_ids.end() || planner_ids.empty()){
    return BT::NodeStatus::FAILURE;
  }
  
  //Remove the planner from the current list 
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node->get_logger(), "Removing planning algorithm %s, since it cannot find a path!", planner_id.c_str());
  planner_ids.erase(itr);
  setOutput("planner_ids", planner_ids);
   
  return BT::NodeStatus::SUCCESS;
}

}  // namespace orunav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orunav2_behavior_tree::RemoveUnfeasiblePlanner>("RemoveUnfeasiblePlanner");
}
