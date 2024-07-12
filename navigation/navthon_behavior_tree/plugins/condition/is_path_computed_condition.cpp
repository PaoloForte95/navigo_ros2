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

#include "navthon_behavior_tree/plugins/condition/is_path_computed_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace navthon{
namespace behavior_tree{

IsPathComputedCondition::IsPathComputedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  first_iteration_ = true;
 
}

BT::NodeStatus IsPathComputedCondition::tick()
{
  
  if(first_iteration_){
    first_iteration_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  
  config().blackboard->get<nav_msgs::msg::Path>("path", path_);

  if (!path_.poses.empty() )
  {
    
     std::cout << "Path length:" << path_.poses.size() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  first_iteration_ = false;
  return BT::NodeStatus::FAILURE;
  
}

}
}  

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navthon::behavior_tree::IsPathComputedCondition>("IsPathComputedCondition");
}
