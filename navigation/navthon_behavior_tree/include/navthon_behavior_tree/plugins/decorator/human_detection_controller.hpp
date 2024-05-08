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

#ifndef NAVTHON_BEHAVIOR_TREE__PLUGINS__DECORATOR__HUMAN_DETECTION_CONTROLLER_HPP_
#define NAVTHON_BEHAVIOR_TREE__PLUGINS__DECORATOR__HUMAN_DETECTION_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "navthon_msgs/msg/human_detection.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"


namespace navthon{
namespace behavior_tree{

/**
 * @brief A BT::DecoratorNode that ticks its child every time the robot
 * is within a certain distance from one of the defined waypoints
 */
class HumanDetectionController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for navthon::behavior_tree::HumanDetectionController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  HumanDetectionController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", std::string("human_detection"), "Human Detection topic name"),
      BT::InputPort<double>("waiting_time", 20, "Waiting time before updating the costmap after a human was detected"),
    
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

   /**
   * @brief Image data callback
   * @param msg Shared pointer to Image message
   */
  void dataCallback(navthon_msgs::msg::HumanDetection msg);


  rclcpp::Node::SharedPtr node_;

  double waiting_time_, last_time_update_;
  std::string topic_;
  rclcpp::Subscription<navthon_msgs::msg::HumanDetection>::SharedPtr data_sub_;
  navthon_msgs::msg::HumanDetection detection_msg_;
  bool human_detected_;
  bool first_time_;
};

}
}  

#endif  // NAVTHON_BEHAVIOR_TREE__PLUGINS__DECORATOR__HUMAN_DETECTION_CONTROLLER_HPP_
