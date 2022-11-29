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

#ifndef ORUNAV2_CORE__GLOBAL_PLANNER_SELECTOR_HPP_
#define ORUNAV2_CORE__GLOBAL_PLANNER_SELECTOR_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace orunav2_core
{

/**
 * @class GlobalPlannerSelector
 * @brief Abstract interface for global planner selectors to adhere to with pluginlib
 */
class GlobalPlannerSelector
{
public:
  using Ptr = std::shared_ptr<GlobalPlannerSelector>;

  /**
   * @brief Virtual destructor
   */
  virtual ~GlobalPlannerSelector() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Select the best global planner from the start to the goal location
   * 
   * @param start Start location
   * @param goal Goal location
   * @return The best global planner  
   */
  virtual std::string selectGlobalPlanner(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};

}  // namespace orunav2_core

#endif  // ORUNAV2_CORE__GLOBAL_PLANNER_SELECTOR_HPP_
