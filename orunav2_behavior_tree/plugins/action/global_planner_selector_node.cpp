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

#include <memory>
#include <string>

#include "orunav2_behavior_tree/plugins/action/global_planner_selector_node.hpp"

namespace orunav2_behavior_tree
{

SelectPlannerAction::SelectPlannerAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
   node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("scan_topic", scan_topic_name_);


  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  gloabal_planner_selector_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_name_,
    rclcpp::QoS(1),
    std::bind(&SelectPlannerAction::callbackGlobalPlannerSelector, this, std::placeholders::_1),
    sub_option);
}


BT::NodeStatus SelectPlannerAction::tick()
{
   callback_group_executor_.spin_some();

  // This behavior always use the last selected planner received from the topic input.
  // When no input is specified it uses the default planner.
  // If the default planner is not specified then we work in "required planner mode":
  // In this mode, the behavior returns failure if the planner selection is not received from
  // the topic input.
  if (last_selected_planner_.empty()) {
    std::string default_planner;
    getInput("default_planner", default_planner);
    if (default_planner.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_planner_ = default_planner;
    }
  }

  setOutput("selected_planner", last_selected_planner_);

  return BT::NodeStatus::SUCCESS;
}

void
SelectPlannerAction::callbackGlobalPlannerSelector(const sensor_msgs::msg::LaserScan::SharedPtr msg)

{
  auto scan_data = msg->ranges;
  auto min = *std::min_element(scan_data.begin(),scan_data.end());
  if (min <= min_distance){
    //last_selected_planner_ = "2d_spline_motion_planner";
    last_selected_planner_ = "GridBased";
  }
  else{
    last_selected_planner_ = "GridBased";
    //last_selected_planner_ = "lattice_based_motion_planner";
  }

 
}

}  // namespace orunav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orunav2_behavior_tree::SelectPlannerAction>("GlobalPlannerSelector");
}
