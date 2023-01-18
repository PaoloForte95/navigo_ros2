// Copyright (c) 2022, Paolo Forte
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
// limitations under the License. Reserved.

#ifndef ORUNAV2_SELECTOR__COSTMAP_SELECTOR
#define ORUNAV2_SELECTOR__COSTMAP_SELECTOR

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "orunav2_core/global_planner_selector.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav2_smac_planner/costmap_downsampler.hpp"

namespace orunav2_selector
{

class CostmapSelector : public orunav2_core::GlobalPlannerSelector
{
public:
  /**
   * @brief constructor
   */
  CostmapSelector();

  /**
   * @brief destructor
   */
  ~CostmapSelector();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Select the best global planner to compute a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return std::string The best global planner
   */
  std::string selectGlobalPlanner(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  nav2_costmap_2d::Costmap2D * _costmap;
  std::unique_ptr<nav2_smac_planner::CostmapDownsampler> _costmap_downsampler;
  int _downsampling_factor;
  bool _downsample_costmap;
  bool _allow_unknown;
  double _distance_threshold;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("CostmapSelector")};
  std::string _global_frame, _name;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
};

}  // namespace orunav2_selector

#endif  // ORUNAV2_SELECTOR__COSTMAP_SELECTOR
