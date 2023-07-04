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

#ifndef NAVTHON_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PLANNER_EQUAL_CONDITION_HPP_
#define NAVTHON_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PLANNER_EQUAL_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace navthon{
namespace behavior_tree{

/**
 * @brief A BT::ConditionNode that check if the actual planner defined in the blackboard is equal to the one provided as an input 
 * returns SUCCESS when they are equal and FAILURE otherwise
 */
class IsPlannerEqualCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for navthon::behavior_tree::IsBatteryLowCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsPlannerEqualCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsPlannerEqualCondition() = delete;

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
      BT::InputPort<std::string>(
        "planner_to_check", "Planner to check")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string planner_id_;
  std::string actual_planner_;
  bool first_iteration_;
};

}
}  

#endif  // NAVTHON_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PLANNER_LATTICE_BASED_CONDITION_HPP_
