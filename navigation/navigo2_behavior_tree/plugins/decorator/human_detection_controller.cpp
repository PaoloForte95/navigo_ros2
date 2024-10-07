// Copyright (c) 2024 Paolo Forte
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

#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "tf2_ros/buffer.h"


#include "behaviortree_cpp_v3/decorator_node.h"

#include <nav2_costmap_2d/array_parser.hpp>

#include "navigo2_behavior_tree/plugins/decorator/human_detection_controller.hpp"

namespace navigo2{
namespace behavior_tree{

HumanDetectionController::HumanDetectionController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  human_detected_(false), last_time_update_(0.0), first_time_(false)
{
rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default
  detection_msg_ = navigo2_msgs::msg::HumanDetection();
  detection_msg_.human_detected = false;
  getInput("topic", topic_);
  getInput("waiting_time", waiting_time_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  data_sub_ = node_->create_subscription<navigo2_msgs::msg::HumanDetection>(topic_, scan_qos,std::bind(&HumanDetectionController::dataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Subscribing to topic: %s", topic_.c_str() );

}

inline BT::NodeStatus HumanDetectionController::tick()
{
    if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting position since we're starting a new iteration of
    // the distance controller (moving from IDLE to RUNNING)
    first_time_ = true;
  }
  setStatus(BT::NodeStatus::RUNNING);

    double current_time = node_->get_clock()->now().seconds();
    auto time_diff = current_time - last_time_update_;
    if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) 
    || (human_detected_  && !detection_msg_.human_detected) 
    || (human_detected_  && (time_diff > waiting_time_)))
    {
      if(first_time_){
          first_time_ = false;
          return BT::NodeStatus::SUCCESS;
      }else{
        last_time_update_ = node_->get_clock()->now().seconds();
        RCLCPP_INFO(node_->get_logger(), "Human non more detected! Clearing costmap!");
        human_detected_ = false;
      }

      const BT::NodeStatus child_state = child_node_->executeTick();
      switch (child_state) {
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::SUCCESS:
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
        default:
          return BT::NodeStatus::FAILURE;
      }
    }
  
  

  return status();
}

void HumanDetectionController::dataCallback(navigo2_msgs::msg::HumanDetection msg)
{
  detection_msg_ = msg;
  if(msg.human_detected){
    RCLCPP_INFO(node_->get_logger(), "Detected Human!");
    human_detected_ = true;
  }
  else {RCLCPP_INFO(node_->get_logger(), "Human not detected!");}
}

}
}  

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo2::behavior_tree::HumanDetectionController>("HumanDetectionController");
}
