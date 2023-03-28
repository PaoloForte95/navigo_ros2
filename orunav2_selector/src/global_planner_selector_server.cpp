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
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "orunav2_selector/global_planner_selector_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace orunav2_selector
{

PlannerSelectorServer::PlannerSelectorServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("selector_server", "", options),
  gp_loader_("orunav2_core", "orunav2_core::GlobalPlannerSelector"),
  default_ids_{"CostmapBased"},
  default_types_{"orunav2_selector::CostmapSelector"}
{
  RCLCPP_INFO(get_logger(), "Creating selector server");

  declare_parameter("selector_frequency", 20.0);
  // Declare this node's parameters
  declare_parameter("selector_plugins", default_ids_);
  // Setup the local costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap");

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

PlannerSelectorServer::~PlannerSelectorServer()
{
  selectors_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlannerSelectorServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();
  RCLCPP_INFO(get_logger(), "Configuring selector interface");


  get_parameter("selector_plugins", selector_ids_);
  if (selector_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_ids_[i]));
    }
  }
  selector_types_.resize(selector_ids_.size());

  get_parameter("selector_frequency", selector_frequency_);
  RCLCPP_INFO(get_logger(), "Selector frequency set to %.4fHz", selector_frequency_);

  
  costmap_ros_->configure();
  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  selector_types_.resize(selector_ids_.size());


  for (size_t i = 0; i != selector_ids_.size(); i++) {
    try {
      selector_types_[i] = nav2_util::get_plugin_type_param(
        node, selector_ids_[i]);
      orunav2_core::GlobalPlannerSelector::Ptr planner_selector =
        gp_loader_.createUniqueInstance(selector_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created global planner selector plugin %s of type %s",
        selector_ids_[i].c_str(), selector_types_[i].c_str());
      planner_selector->configure(node, selector_ids_[i], costmap_ros_->getTfBuffer(), costmap_ros_);
      selectors_.insert({selector_ids_[i], planner_selector});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create the global planner selector. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != selector_ids_.size(); i++) {
    selector_ids_concat_ += selector_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Global Planner Selector Server has %s selectors available.", selector_ids_concat_.c_str());

 
  // Initialize pubs & subs
  planner_publisher_ = create_publisher<std_msgs::msg::String>("selected_planner", 1);

  // Create the action servers for path planning to a pose and through poses
  action_server_select_ = std::make_unique<ActionServerSelect>(
    shared_from_this(),
    "select_global_planner",
    std::bind(&PlannerSelectorServer::computePlanner, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerSelectorServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  planner_publisher_->on_activate();
  action_server_select_->activate();
  costmap_ros_->activate();

  PlannerMap::iterator it;
  for (it = selectors_.begin(); it != selectors_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();

  is_planner_valid_service_ = node->create_service<orunav2_msgs::srv::IsPlannerValid>(
    "is_path_valid",
    std::bind(
      &PlannerSelectorServer::isPlannerValid, this,
      std::placeholders::_1, std::placeholders::_2));

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PlannerSelectorServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerSelectorServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_select_->deactivate();
  planner_publisher_->on_deactivate();
  costmap_ros_->deactivate();

  PlannerMap::iterator it;
  for (it = selectors_.begin(); it != selectors_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerSelectorServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_select_.reset();
  planner_publisher_.reset();
  costmap_ros_->cleanup();

  PlannerMap::iterator it;
  for (it = selectors_.begin(); it != selectors_.end(); ++it) {
    it->second->cleanup();
  }
  selectors_.clear();
  costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerSelectorServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerSelectorServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void PlannerSelectorServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool PlannerSelectorServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerSelectorServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool PlannerSelectorServer::getStartPose(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal,
  geometry_msgs::msg::PoseStamped & start)
{
  if (goal->use_start) {
    start = goal->start;
  } else if (!costmap_ros_->getRobotPose(start)) {
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerSelectorServer::transformPosesToGlobalFrame(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  geometry_msgs::msg::PoseStamped & curr_start,
  geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
  {
    RCLCPP_WARN(
      get_logger(), "Could not transform the start or goal pose in the costmap frame");
    action_server->terminate_current();
    return false;
  }

  return true;
}


void
PlannerSelectorServer::computePlanner()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_select_->get_current_goal();
  auto result = std::make_shared<ActionSelect::Result>();

  try {
    if (isServerInactive(action_server_select_) || isCancelRequested(action_server_select_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_select_, goal);

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_select_, goal, start)) {
      return;
    }

    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (!transformPosesToGlobalFrame(action_server_select_, start, goal_pose)) {
      return;
    }
    RCLCPP_WARN( get_logger(), "Size Planners : %d ", goal->planner_ids.size());

    result->planner_id = getPlanner(start, goal_pose, goal->selector_id,  goal->planner_ids);
    auto message = std_msgs::msg::String();
    message.data = result->planner_id;
    // Publish the plan for visualization purposes
    publishPlanner(message);

    action_server_select_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to select planner to (%.2f, %.2f): \"%s\"",
      goal->selector_id.c_str(), goal->goal.pose.position.x,
      goal->goal.pose.position.y, ex.what());
    action_server_select_->terminate_current();
  }
}

std::string
PlannerSelectorServer::getPlanner(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & selector_id,
  const std::vector<std::string> & planner_ids)
{
  RCLCPP_DEBUG(
    get_logger(), "Attempting to a select a planner to go from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  if (selectors_.find(selector_id) != selectors_.end()) {
    return selectors_[selector_id]->selectGlobalPlanner(start, goal, planner_ids);
  } else {
    if (selectors_.size() == 1 && selector_id.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", selector_ids_concat_.c_str());
      return selectors_[selectors_.begin()->first]->selectGlobalPlanner(start, goal, planner_ids);
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner %s is not a valid planner. "
        "Planner names are: %s", selector_id.c_str(),
        selector_ids_concat_.c_str());
    }
  }

  return std::string();
}

void
PlannerSelectorServer::publishPlanner(const std_msgs::msg::String & msg)
{
  auto message = std::make_unique<std_msgs::msg::String>(msg);
  if (planner_publisher_->is_activated() && planner_publisher_->get_subscription_count() > 0) {
    planner_publisher_->publish(std::move(message));
  }
}

void PlannerSelectorServer::isPlannerValid(
  const std::shared_ptr<orunav2_msgs::srv::IsPlannerValid::Request> request,
  std::shared_ptr<orunav2_msgs::srv::IsPlannerValid::Response> response)
{
  response->is_valid = true;

  if (request->planner.empty()) {
    response->is_valid = false;
    return;
  }

  //Check if selected planner if part of the supported planner

}

rcl_interfaces::msg::SetParametersResult
PlannerSelectorServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
    }
  }

  result.successful = true;
  return result;
}

}  // namespace orunav2_selector

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(orunav2_selector::PlannerSelectorServer)
