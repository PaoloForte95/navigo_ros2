// Copyright (c) 2022, Paolo Forte
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>

#include "orunav2_selector/costmap_selector.hpp"
#include "nav2_util/geometry_utils.hpp"

// #define BENCHMARK_TESTING

namespace orunav2_selector
{
using namespace std::chrono;  // NOLINT
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

CostmapSelector::CostmapSelector() :
  _costmap(nullptr),
  _costmap_downsampler(nullptr)
{
}

CostmapSelector::~CostmapSelector()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type CostmapSelector",
    _name.c_str());
}

void CostmapSelector::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  auto node = parent.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(_logger, "Configuring %s of type CostmapSelector", name.c_str());

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.125));
  _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".downsampling_factor", _downsampling_factor);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", _allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".distance_threshold", rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".distance_threshold", _distance_threshold);




  // Initialize costmap downsampler
  if (_downsample_costmap && _downsampling_factor > 1) {
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler = std::make_unique<nav2_smac_planner::CostmapDownsampler>();
    _costmap_downsampler->on_configure(
      node, _global_frame, topic_name, _costmap, _downsampling_factor);
  }


  RCLCPP_INFO(
    _logger, "Configured plugin %s of type CostmapSelector with "
    "tolerance %.2f, and %s.",
    _name.c_str(), _tolerance,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal");
}

void CostmapSelector::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type CostmapSelector",
    _name.c_str());
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&CostmapSelector::dynamicParametersCallback, this, _1));
}

void CostmapSelector::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type CostmapSelector",
    _name.c_str());
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
  _dyn_params_handler.reset();
}

void CostmapSelector::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type CostmapSelector",
    _name.c_str());
  if (_costmap_downsampler) {
    _costmap_downsampler->on_cleanup();
    _costmap_downsampler.reset();
  }
}

std::string CostmapSelector::selectGlobalPlanner(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
  }

  // Set starting point
  unsigned int mx_start, my_start, mx_goal, my_goal;
  costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start);

  // Set goal point
  costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);

  // Setup message
  std::string selected_planner;


  //Get information from the costmap

  // Corner case of start and goal beeing on the same cell
  if (mx_start == mx_goal && my_start == my_goal) {
    if (costmap->getCost(mx_start, my_start) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(_logger, "It Is Fine Now. Why? Because I Am Here!");
      return selected_planner;
    }
    return selected_planner;
  }
  unsigned int dist;
  dist = costmap->cellDistance(_distance_threshold);
  RCLCPP_WARN_STREAM(_logger, "Distance cell: " << dist);



#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif


  return selected_planner;
}

rcl_interfaces::msg::SetParametersResult
CostmapSelector::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  bool reinit_downsampler = false;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == _name + ".tolerance") {
        _tolerance = static_cast<float>(parameter.as_double());
      } 
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == _name + ".downsample_costmap") {
        reinit_downsampler = true;
        _downsample_costmap = parameter.as_bool();
      } else if (name == _name + ".allow_unknown") {
        _allow_unknown = parameter.as_bool();
      } else if (name == _name + ".use_final_approach_orientation") {
        
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == _name + ".downsampling_factor") {
        reinit_downsampler = true;
        _downsampling_factor = parameter.as_int();
      } 
       
    }
  }
  // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
  if (reinit_downsampler) {
    // Re-Initialize costmap downsampler
    if (reinit_downsampler) {
      if (_downsample_costmap && _downsampling_factor > 1) {
        auto node = _node.lock();
        std::string topic_name = "downsampled_costmap";
        _costmap_downsampler = std::make_unique<nav2_smac_planner::CostmapDownsampler>();
        _costmap_downsampler->on_configure(
          node, _global_frame, topic_name, _costmap, _downsampling_factor);
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace orunav2_selector

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(orunav2_selector::CostmapSelector, orunav2_core::GlobalPlannerSelector)
