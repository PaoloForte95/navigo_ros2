// Copyright (c) 2023 paolo Forte
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

#ifndef ORUNAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SEGMENTED_AREA_CONTROLLER_HPP_
#define ORUNAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SEGMENTED_AREA_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"


namespace orunav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child every time the robot
 * enters a new area
 */
class SegmentedAreaController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for orunav2_behavior_tree::SegmentedAreaController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  SegmentedAreaController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("area_state_topic", std::string("area_state"), "Area state topic"),
      BT::InputPort<int>("initial_value", -1 , "Initial state value"),
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Callback function for area state topic
   * @param msg Shared pointer to std_msgs::msg::Int32 message
   */
  void AreaStateCallback(std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::string area_state_topic_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr area_state_sub_;

  int previous_value_;
  int current_area_value_;
  bool first_time_;
};

}  // namespace orunav2_behavior_tree

#endif  // ORUNAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SEGMENTED_AREA_CONTROLLER_HPP_
