// Copyright (c) 2021 Paolo Forte
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

#ifndef ORUNAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PLAN_ACTION_HPP_
#define ORUNAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PLAN_ACTION_HPP_

#include <string>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "orunav2_msgs/action/select_planner.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace orunav2_behavior_tree
{

/**
 * @brief The PlannerSelector behavior is used to switch the planner
 * that will be used by the planner server. It uses sensors information 
 * to get the decision about what planner must be used. It is usually used before of
 * the ComputePathToPoseAction. The selected_planner output port is passed to planner_id
 * input port of the ComputePathToPoseAction
 */
class SelectPlannerAction : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for orunav2_behavior_tree::SelectPlanner
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  SelectPlannerAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return 
      {
        BT::InputPort<std::string>(
        "default_planner",
        "the default planner to use if there is not any external topic message received."),

        BT::InputPort<std::string>(
        "scan_topic",
        "scan",
        "the topic name of the sensor scan"),
        
        BT::OutputPort<std::string>("selected_planner", "The selected global path planner"),
      };
  }


  private:

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief callback function for the global_planner_selector action
   *
   * @param msg the message with the local costamp of the robot
   */
  void callbackGlobalPlannerSelector(const sensor_msgs::msg::LaserScan::SharedPtr msg);


  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr gloabal_planner_selector_sub_;

  std::string last_selected_planner_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string scan_topic_name_;

  double min_distance = 2.0;

  
};

}  // namespace orunav2_behavior_tree

#endif  // ORUNAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PLAN_ACTION_HPP_
