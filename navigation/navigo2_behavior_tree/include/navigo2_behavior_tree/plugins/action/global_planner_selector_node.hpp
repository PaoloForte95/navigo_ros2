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

#ifndef NAVIGO2_BEHAVIOR_TREE__PLUGINS__ACTION__SELECT_GLOBAL_PLANNER_ACTION_HPP_
#define NAVIGO2_BEHAVIOR_TREE__PLUGINS__ACTION__SELECT_GLOBAL_PLANNER_ACTION_HPP_

#include <string>

#include "navigo2_msgs/action/global_planner_selector.hpp"
#include "navigo2_behavior_tree/bt_action_node.hpp"

namespace navigo2{
namespace behavior_tree{

/**
 * @brief A navigo2::behavior_tree::BtActionNode class that wraps navigo2_msgs::action::GlobalPlannerSelector
 */
class SelectGlobalPlannerAction : public BtActionNode<navigo2_msgs::action::GlobalPlannerSelector>
{
public:

  typedef std::vector<std::string> Planners;

  /**
   * @brief A constructor for navigo2::behavior_tree::SelectGlobalPlannerAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SelectGlobalPlannerAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancelation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<std::string>("selected_planner", "Path planner selected by SelectGlobalPlanner node"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start", "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>(
          "selector_id", "",
          "Mapped name to the selector plugin type to use"),
        BT::InputPort<Planners>("planner_ids", "The list of planners to select from"),
      });
  }
};

}
}  

#endif  // NAVIGO2_BEHAVIOR_TREE__PLUGINS__ACTION__SELECT_GLOBAL_PLANNER_ACTION_HPP_
