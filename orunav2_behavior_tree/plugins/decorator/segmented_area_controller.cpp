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

#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "tf2_ros/buffer.h"


#include "behaviortree_cpp_v3/decorator_node.h"

#include <nav2_costmap_2d/array_parser.hpp>

#include "orunav2_behavior_tree/plugins/decorator/segmented_area_controller.hpp"

namespace orunav2_behavior_tree
{

SegmentedAreaController::SegmentedAreaController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  area_state_topic_("area_state"),
  first_time_(false),
  previous_value_(-1)
{
  getInput("area_state_topic", area_state_topic_);
  getInput("initial_value", previous_value_ );
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");


  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  area_state_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    area_state_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&SegmentedAreaController::AreaStateCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_DEBUG(node_->get_logger(), "Initialized an SegmentedAreaController BT node");
}

inline BT::NodeStatus SegmentedAreaController::tick()
{
  callback_group_executor_.spin_some();
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting position since we're starting a new iteration of
    // the distance controller (moving from IDLE to RUNNING)
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);


  
    // The child gets ticked the first time through and every time the area state changed.
    // In addition, once the child begins to run, it is
    // ticked each time 'til completion
    if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
      current_area_value_ != previous_value_)
    {
      if(first_time_){
          first_time_ = false;
      }else{
        
        RCLCPP_INFO(node_->get_logger(), "Entering a new area! Previous area state: %d, Current area state: %d",previous_value_, current_area_value_);
        previous_value_ = current_area_value_;
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
    else {
     
    }

  return status();
}


void SegmentedAreaController::AreaStateCallback(std_msgs::msg::Int32::SharedPtr msg)
{
  current_area_value_ = msg->data;
  RCLCPP_INFO(node_->get_logger(), "Current Area Value %d:",current_area_value_);
}


}  // namespace orunav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orunav2_behavior_tree::SegmentedAreaController>("SegmentedAreaController");
}
