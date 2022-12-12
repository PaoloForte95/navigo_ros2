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

#ifndef ORUNAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
#define ORUNAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_

#include <string>
#include <memory>

#include "orunav2_msgs/action/follow_path.hpp"
#include "orunav2_behavior_tree/bt_action_node.hpp"

namespace orunav2_behavior_tree
{

/**
 * @brief A orunav2_behavior_tree::BtActionNode class that wraps orunav2_msgs::action::FollowPath
 */
class FollowPathAction : public BtActionNode<orunav2_msgs::action::FollowPath>
{
public:
  /**
   * @brief A constructor for orunav2_behavior_tree::FollowPathAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  FollowPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet
   * @param feedback shared_ptr to latest feedback message
   */
  void on_wait_for_result(
    std::shared_ptr<const orunav2_msgs::action::FollowPath::Feedback> feedback) override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<orunav2_msgs::msg::Path>("path", "Path to follow"),
        BT::InputPort<std::string>("controller_id", ""),
        BT::InputPort<std::string>("goal_checker_id", ""),
      });
  }
};

}  // namespace orunav2_behavior_tree

#endif  // ORUNAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
