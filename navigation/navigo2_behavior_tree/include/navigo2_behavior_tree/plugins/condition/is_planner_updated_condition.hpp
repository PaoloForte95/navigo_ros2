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

#ifndef  NAVIGO2_BEHAVIOR_TREE__PLUGINS__CONDITION__UPDATED_PLANNER_CONDITION_HPP_
#define  NAVIGO2_BEHAVIOR_TREE__PLUGINS__CONDITION__UPDATED_PLANNER_CONDITION_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace navigo2{
namespace behavior_tree{
/**
 * @brief A BT::ConditionNode that returns SUCCESS when planner is
 * updated on the blackboard and FAILURE otherwise
 */
class PlannerUpdatedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for navigo2::behavior_tree::PlannerUpdatedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  PlannerUpdatedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  PlannerUpdatedCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;


  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
       BT::InputPort<std::string>("planner", "Planner to check"),

    };
  }

private:

  rclcpp::Node::SharedPtr node_;
  std::string previous_planner_;
};

}
}  


#endif  // NAVIGO2_BEHAVIOR_TREE__PLUGINS__CONDITION__UPDATED_PLANNER_CONDITION_HPP_
