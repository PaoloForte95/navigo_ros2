// Copyright (c) 2023 Paolo Forte

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

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "navigo2_behavior_tree/plugins/action/add_planners_action.hpp"

namespace navigo2{
namespace behavior_tree{

AddPlanners::AddPlanners(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  getInput("input_planners", string_planners_to_add);
  std::stringstream ss(string_planners_to_add);
  while( ss.good() )
    {
        std::string substr;
        getline( ss, substr, ',' );
        planners_to_add.push_back( substr );
    }
    config().blackboard->set<std::vector<std::string>>("planner_ids",planners_to_add);
}

inline BT::NodeStatus AddPlanners::tick()
{
    setStatus(BT::NodeStatus::RUNNING);
    config().blackboard->get<std::vector<std::string>>("planner_ids", actual_planners_);
    auto node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    Planners goal_planner_ids;
    
    if (planners_to_add.empty() && actual_planners_.empty()) {
        setOutput("output_planners", goal_planner_ids);
        return BT::NodeStatus::FAILURE;
    }
    //Add the planners
    for(auto planner : planners_to_add){
        auto itr = std::find(actual_planners_.begin(), actual_planners_.end(), planner);
        //Add only if absent
        if (itr == actual_planners_.end()){
            RCLCPP_INFO(node_->get_logger(), "Restoring planning algorithm: %s!", planner.c_str());
            goal_planner_ids.push_back(planner);
        }
        else {
            RCLCPP_INFO(node_->get_logger(), "Planning algorithm %s is already present in the list! No need to restore it!", planner.c_str());
        }

    }
    
    //Add the planner currently in the blackboard
    for(auto planner : actual_planners_){
        auto itr = std::find(goal_planner_ids.begin(), goal_planner_ids.end(), planner);
        if (itr == goal_planner_ids.end()){
            goal_planner_ids.push_back(planner);
        }
    }

    setOutput("output_planners", goal_planner_ids);
    return BT::NodeStatus::SUCCESS;
}

}
}  

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo2::behavior_tree::AddPlanners>("AddPlanners");
}
