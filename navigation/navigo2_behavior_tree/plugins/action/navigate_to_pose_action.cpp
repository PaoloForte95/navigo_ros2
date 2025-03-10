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

#include "navigo2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"

namespace navigo2{
namespace behavior_tree{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<navigo2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void NavigateToPoseAction::on_tick()
{
  if (!getInput("goal", goal_.pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateToPoseAction: goal not provided");
    return;
  }
  getInput("behavior_tree", goal_.behavior_tree);
}

}
}  

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<navigo2::behavior_tree::NavigateToPoseAction>(
        name, "ecl_navigate_to_pose", config);
    };

  factory.registerBuilder<navigo2::behavior_tree::NavigateToPoseAction>(
    "NavigateToPose", builder);
}
