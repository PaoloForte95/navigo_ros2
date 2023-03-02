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
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include <orunav2_smac_planner/planners/smac_planner_lattice.hpp>
#include <orunav2_generic/types.hpp>
#include <orunav2_generic/path_utils.hpp>
#include <orunav2_conversions/conversions.hpp>
#include "orunav2_smac_planner/nav_msgs_occupancy_grid_to_planner_map.hpp"



#include <ecceleron/base/common/pose_utilities.h>
#include <ecceleron_conversions/conversions.hpp>
// #define BENCHMARK_TESTING



namespace orunav2_smac_planner
{


using namespace std::chrono;  // NOLINT
using rcl_interfaces::msg::ParameterType;

SmacPlannerLattice::SmacPlannerLattice()
: 
  _smoother(nullptr),
  _costmap(nullptr),
  _model(nullptr),
  _path_finder(nullptr)
{
}

SmacPlannerLattice::~SmacPlannerLattice()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlannerLattice",
    _name.c_str());
}

void SmacPlannerLattice::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  auto node = parent.lock();
  _tf = tf;
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();
  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  RCLCPP_INFO(_logger, "Configuring %s of type SmacPlannerLattice", name.c_str());

  // General planner params
  double analytic_expansion_max_length_m;
  bool smooth_path;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.25));
  _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", _allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
  node->get_parameter(name + ".max_iterations", _max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
  node->get_parameter(name + ".max_on_approach_iterations", _max_on_approach_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", _use_final_approach_orientation);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_path", smooth_path);

  // Default to an ackermann model
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_primitives_directory", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("orunav2_smac_planner") + 
      "/lattice_primitives/Volvo_L70/"));   
  node->get_parameter(name + ".motion_primitives_directory", _search_info.lattice_filepath);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lookup_tables_directory", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("orunav2_smac_planner") + 
      "/lattice_lookup_tables/output.json"));   
  node->get_parameter(name + ".lookup_tables_directory", _search_info.lookup_tables_filepath);
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".maps_directory", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("orunav2_smac_planner")));   
  node->get_parameter(name + ".maps_directory", _search_info.maps_directory);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".model", rclcpp::ParameterValue("volvo_l70_all.reduced"));   
  node->get_parameter(name + ".model", model);
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".min_incr_path_dist", rclcpp::ParameterValue(0.2));   
  node->get_parameter(name + ".min_incr_path_dist", _min_incr_path_dist);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".save_paths", rclcpp::ParameterValue(false));   
  node->get_parameter(name + ".save_paths", _save_paths);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_time", rclcpp::ParameterValue(10.0));
  node->get_parameter(name + ".max_planning_time", _max_planning_time);

  nav2_util::declare_parameter_if_not_declared(node, name + ".front_frame_id", rclcpp::ParameterValue("front_link_axis"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".rear_frame_id", rclcpp::ParameterValue("rear_link_axis"));
  _front_frame_id = node->get_parameter(name + ".front_frame_id").as_string();
  _rear_frame_id = node->get_parameter(name + ".rear_frame_id").as_string();
  _search_info.minimum_turning_radius =
    _metadata.min_turning_radius / (_costmap->getResolution());
  
  
  _motion_model = MotionModel::STATE_LATTICE;


    WP::setPrimitivesDir(_search_info.lattice_filepath);
    WP::setTablesDir(_search_info.lookup_tables_filepath);
    WP::setMapsDir(_search_info.maps_directory);
    RCLCPP_INFO_STREAM(_logger, "[SmacPlannerLattice] - Setting primitive dir : " << _search_info.lattice_filepath );
    RCLCPP_INFO_STREAM(_logger, "[SmacPlannerLattice] - Maps dir : " << _search_info.maps_directory );
  _model = new LHDModel(model);
  RCLCPP_INFO_STREAM(_logger, "[SmacPlannerLattice] - Using model : " << model );
  if (_max_on_approach_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  float lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution());

  // Make sure its a whole number
  lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(lookup_table_dim) % 2 == 0) {
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      lookup_table_dim);
    lookup_table_dim += 1.0;
  }

  

  //_collision_checker = orunav2_smac_planner::CollisionDetector(_costmap);
  //_collision_checker.setFootprint(
    //costmap_ros->getRobotFootprint(),
    //costmap_ros->getUseRadius(),
    //nav2_smac_planner::findCircumscribedCost(costmap_ros));

  // Initialize A* template
    //_a_star = std::make_unique<nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeLattice>>(_motion_model, _search_info);
    //_a_star->initialize(
     // _allow_unknown,
     // _max_iterations,
     // _max_on_approach_iterations,
     // _max_planning_time,
      //lookup_table_dim,
      //_metadata.number_of_headings);

  // Initialize path smoother
  if (smooth_path) {
    nav2_smac_planner::SmootherParams params;
    params.get(node, name);
    _smoother = std::make_unique<nav2_smac_planner::Smoother>(params);
    _smoother->initialize(_metadata.min_turning_radius);
  }

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlannerLattice with "
    "maximum iterations %i, max on approach iterations %i, "
    "and %s. Tolerance %.2f. Using motion model: %s. State lattice file: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str(), _search_info.lattice_filepath.c_str());
}

void SmacPlannerLattice::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SmacPlannerLattice::dynamicParametersCallback, this, std::placeholders::_1));
}

void SmacPlannerLattice::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  _dyn_params_handler.reset();
}

void SmacPlannerLattice::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _smoother.reset();
  _raw_plan_publisher.reset();
}

nav_msgs::msg::Path SmacPlannerLattice::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_INFO(_logger, "Computing Global Plan with Lattice-Based Algorithm");
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  //Information about start and goal
  double steering_start= 0.0;
  double steering_goal = 0.0;

  if (!getArticulatedJointAngle(steering_start)) {
        RCLCPP_ERROR(_logger, "failed to query the articulated joint angle, check front_frame_id and rear_frame_id");
  }

  RCLCPP_INFO(_logger, "[SmacPlannerLattice] - start : [%f,%f,%f](%f)", start.pose.position.x, start.pose.position.y,tf2::getYaw(start.pose.orientation), steering_start);
  RCLCPP_INFO(_logger, "[SmacPlannerLattice] - goal :  [%f,%f,%f](%f)", goal.pose.position.x, goal.pose.position.y,tf2::getYaw(goal.pose.orientation), steering_goal);
    

  double map_offset_x = _costmap->getOriginX();
  double map_offset_y = _costmap->getOriginY();
  double start_orientation = tf2::getYaw(start.pose.orientation);
  double goal_orientation = tf2::getYaw(goal.pose.orientation);


  WorldOccupancyMap planner_map;
  RCLCPP_INFO(_logger, "[SmacPlannerLattice] - Costmap Resolution : %f", _costmap->getResolution());
  RCLCPP_INFO(_logger, "[SmacPlannerLattice] - Model Resolution : %f", _model->getModelGranularity());
  convertNavMsgsOccupancyGridToWorldOccupancyMapRef(_costmap, planner_map);

  // Setup message
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  
  if (planner_map.getMap().empty()) {
    RCLCPP_ERROR(_logger, "[SmacPlannerLattice] - error in the provided map / conversion");
    return plan;
  }

  if (planner_map.getMap().empty()){
      _path_finder = new PathFinder(20, 20);
       RCLCPP_ERROR(_logger, "[SmacPlannerLattice] - Creating PathFinder with empty map");
      }
    else{
      _path_finder = new PathFinder(planner_map);
       RCLCPP_ERROR(_logger, "[SmacPlannerLattice] - Creating PathFinder with map");
      }
    
  if (_max_planning_time > 0.) 
    _path_finder->setTimeBound(_max_planning_time);

  
  VehicleMission vm(_model,
                      start.pose.position.x-map_offset_x, start.pose.position.y-map_offset_y, start_orientation, 0, //FIXME -> steering not 0 
                      goal.pose.position.x-map_offset_x, goal.pose.position.y-map_offset_y, goal_orientation, 0);


  _path_finder->addMission(&vm);

    
    RCLCPP_INFO(_logger, "[SmacPlannerLattice] - Starting to solve the path planning problem ... ");
    rclcpp::Time start_time = _clock->now();
    std::vector<std::vector<Configuration*> > solution = _path_finder->solve(false);
    rclcpp::Time stop_time = _clock->now();
    RCLCPP_INFO(_logger, "[SmacPlannerLattice] - Starting to solve the path planning problem - done");
    RCLCPP_INFO(_logger, "[SmacPlannerLattice] - PATHPLANNER_PROCESSING_TIME: %f", (stop_time-start_time).seconds());
    
    assert(!solution.empty());
    bool solution_found = (solution[0].size() != 0);
    RCLCPP_INFO_STREAM(_logger, "[SmacPlannerLattice] - solution_found : " << solution_found);
    RCLCPP_INFO_STREAM(_logger, "[SmacPlannerLattice] - solution[0].size() : " << solution[0].size());
    
    orunav2_generic::Path path;
    
    for (std::vector<std::vector<Configuration*> >::iterator it = solution.begin(); it != solution.end(); it++)
    {
      for (std::vector<Configuration*>::iterator confit = (*it).begin(); confit != (*it).end(); confit++) {
        std::vector<vehicleSimplePoint> path_local = (*confit)->getTrajectory();
        
        for (std::vector<vehicleSimplePoint>::iterator it2 = path_local.begin(); it2 != path_local.end(); it2++) {
          double orientation = it2->orient;  
          
          orunav2_generic::State2d state(orunav2_generic::Pose2d(it2->x+map_offset_x,
                                                               it2->y+map_offset_y,
                                                               orientation), it2->steering);
          path.addState2dInterface(state);
          
        }
      }
    }

    RCLCPP_INFO_STREAM(_logger, "[SmacPlannerLattice] - Nb of path points : " << path.sizePath());

    // Cleanup
    for (std::vector<std::vector<Configuration*> >::iterator it = solution.begin(); it != solution.end(); it++) {
      std::vector<Configuration*> confs = (*it);
      for (std::vector<Configuration*>::iterator confit = confs.begin(); confit != confs.end(); confit++) {
        delete *confit;
      }
      confs.clear();
    }
    solution.clear();
    // Cleanup - end

        
    if (path.sizePath() == 0)
      solution_found = false;


   if (solution_found) {
      // First requirement (that the points are separated by a minimum distance).
      orunav2_generic::Path path_min_dist = orunav2_generic::minIncrementalDistancePath(path, _min_incr_path_dist);
      // Second requirment (path states are not allowed to change direction of motion without any intermediate points).
      orunav2_generic::Path path_dir_change = orunav2_generic::minIntermediateDirPathPoints(path_min_dist);

      auto res_path = orunav2_conversions::createPathMsgFromPathInterface(path_dir_change);
      res_path.target_start.pose = start.pose;
      res_path.target_goal.pose = goal.pose;
      for (int i = 0; i < res_path.path.size(); i++){
        geometry_msgs::msg::PoseStamped last_pose;
        last_pose.pose = res_path.path[i].pose;
        plan.poses.push_back(last_pose);
      }


   }

  // Compute plan
  
 /* nav2_smac_planner::NodeLattice::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!_a_star->createPath(
        path, num_iterations, _tolerance / static_cast<float>(_costmap->getResolution())))
    {
      if (num_iterations < _a_star->getMaxIterations()) {
        error = std::string("no valid path found");
      } else {
        error = std::string("exceeded maximum iterations");
      }
    }
  } catch (const std::runtime_error & e) {
    error = "invalid use: ";
    error += e.what();
  }

  if (!error.empty()) {
    RCLCPP_WARN(
      _logger,
      "%s: failed to create plan, %s.",
      _name.c_str(), error.c_str());
    return plan;
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  geometry_msgs::msg::PoseStamped last_pose = pose;
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = nav2_smac_planner::getWorldCoords(path[i].x, path[i].y, _costmap);
    pose.pose.orientation = nav2_smac_planner::getWorldOrientation(path[i].theta);
    if (fabs(pose.pose.position.x - last_pose.pose.position.x) < 1e-4 &&
      fabs(pose.pose.position.y - last_pose.pose.position.y) < 1e-4 &&
      fabs(tf2::getYaw(pose.pose.orientation) - tf2::getYaw(last_pose.pose.orientation)) < 1e-4)
    {
      RCLCPP_DEBUG(
        _logger,
        "Removed a path from the path due to replication. "
        "Make sure your minimum control set does not contain duplicate values!");
      continue;
    }
    last_pose = pose;
    plan.poses.push_back(pose);
  }
  */

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan);
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  //if (_smoother && num_iterations > 1) {
  if (_smoother &&  solution_found ) {
    _smoother->smooth(plan, _costmap, time_remaining);
  }
// If use_final_approach_orientation=true, interpolate the last pose orientation from the
  // previous pose to set the orientation to the 'final approach' orientation of the robot so
  // it does not rotate.
  // And deal with corner case of plan of length 1
  // If use_final_approach_orientation=false (default), override last pose orientation to match goal
  size_t plan_size = plan.poses.size();
  if (_use_final_approach_orientation && solution_found) {
    if (plan_size == 1) {
      plan.poses.back().pose.orientation = start.pose.orientation;
    } else if (plan_size > 1) {
      double dx, dy, theta;
      auto last_pose = plan.poses.back().pose.position;
      auto approach_pose = plan.poses[plan_size - 2].pose.position;
      dx = last_pose.x - approach_pose.x;
      dy = last_pose.y - approach_pose.y;
      theta = atan2(dy, dx);
      plan.poses.back().pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(theta);
    }
  } else if (plan_size > 0) {
    plan.poses.back().pose.orientation = goal.pose.orientation;
  }

return plan;
}

  bool SmacPlannerLattice::getArticulatedJointAngle(double &angle) const
  {
    // Front pose and rear pose
    // Utilize tf to be more generic towards simulation vs. real systems.
    if (!_tf) {
      RCLCPP_ERROR(_logger,"tf not initiated(!)");
      return false;
    }
    try
    {
      // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
      geometry_msgs::msg::TransformStamped joint = _tf->lookupTransform(_front_frame_id, _rear_frame_id, rclcpp::Time(0));
      
      ecl::PoseVector2d joint_angle2d = ecl::base::conversions::toPoseVector2d(ecl::base::conversions::toPoseMsg(joint.transform));
      angle = joint_angle2d[2];
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(_logger,"Cannot get the articulated angle as no transform is available: %s\n", ex.what());
      return false;
    }
    return true;  
  }

rcl_interfaces::msg::SetParametersResult
SmacPlannerLattice::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  bool reinit_lattice_planner = false;
  bool reinit_smoother = false;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == _name + ".max_planning_time") {
        reinit_lattice_planner = true;
        _max_planning_time = parameter.as_double();
      } else if (name == _name + ".tolerance") {
        _tolerance = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".lookup_table_size") {
        reinit_lattice_planner = true;
        _lookup_table_size = parameter.as_double();
      } else if (name == _name + ".reverse_penalty") {
        reinit_lattice_planner = true;
        _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".change_penalty") {
        reinit_lattice_planner = true;
        _search_info.change_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".non_straight_penalty") {
        reinit_lattice_planner = true;
        _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".cost_penalty") {
        reinit_lattice_planner = true;
        _search_info.cost_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".rotation_penalty") {
        reinit_lattice_planner = true;
        _search_info.rotation_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_ratio") {
        reinit_lattice_planner = true;
        _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_max_length") {
        reinit_lattice_planner = true;
        _search_info.analytic_expansion_max_length =
          static_cast<float>(parameter.as_double()) / _costmap->getResolution();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == _name + ".allow_unknown") {
        reinit_lattice_planner = true;
        _allow_unknown = parameter.as_bool();
      } else if (name == _name + ".cache_obstacle_heuristic") {
        reinit_lattice_planner = true;
        _search_info.cache_obstacle_heuristic = parameter.as_bool();
      } else if (name == _name + ".allow_reverse_expansion") {
        reinit_lattice_planner = true;
        _search_info.allow_reverse_expansion = parameter.as_bool();
      } else if (name == _name + ".smooth_path") {
        if (parameter.as_bool()) {
          reinit_smoother = true;
        } else {
          _smoother.reset();
        }
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == _name + ".max_iterations") {
        reinit_lattice_planner = true;
        _max_iterations = parameter.as_int();
        if (_max_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "maximum iteration selected as <= 0, "
            "disabling maximum iterations.");
          _max_iterations = std::numeric_limits<int>::max();
        }
      }
    } else if (name == _name + ".max_on_approach_iterations") {
      reinit_lattice_planner = true;
      _max_on_approach_iterations = parameter.as_int();
      if (_max_on_approach_iterations <= 0) {
        RCLCPP_INFO(
          _logger, "On approach iteration selected as <= 0, "
          "disabling tolerance and on approach iterations.");
        _max_on_approach_iterations = std::numeric_limits<int>::max();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == _name + ".lattice_filepath") {
        reinit_lattice_planner = true;
        if (_smoother) {
          reinit_smoother = true;
        }
        _search_info.lattice_filepath = parameter.as_string();
        _metadata = nav2_smac_planner::LatticeMotionTable::getLatticeMetadata(_search_info.lattice_filepath);
        _search_info.minimum_turning_radius =
          _metadata.min_turning_radius / (_costmap->getResolution());
      }
    }
  }

  // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
  if (reinit_lattice_planner || reinit_smoother) {
    // convert to grid coordinates
    _search_info.minimum_turning_radius =
      _metadata.min_turning_radius / (_costmap->getResolution());
    float lookup_table_dim =
      static_cast<float>(_lookup_table_size) /
      static_cast<float>(_costmap->getResolution());

    // Make sure its a whole number
    lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));

    // Make sure its an odd number
    if (static_cast<int>(lookup_table_dim) % 2 == 0) {
      RCLCPP_INFO(
        _logger,
        "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
        lookup_table_dim);
      lookup_table_dim += 1.0;
    }

    // Re-Initialize smoother
    if (reinit_smoother) {
      auto node = _node.lock();
      nav2_smac_planner::SmootherParams params;
      params.get(node, _name);
      _smoother = std::make_unique<nav2_smac_planner::Smoother>(params);
      _smoother->initialize(_metadata.min_turning_radius);

    }

    // Re-Initialize A* template
    //if (reinit_a_star) {
     // _a_star = std::make_unique<nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeLattice>>(_motion_model, _search_info);
     // _a_star->initialize(
        //_allow_unknown,
        //_max_iterations,
        //_max_on_approach_iterations,
        //_max_planning_time,
        //lookup_table_dim,
        //_metadata.number_of_headings);
    //}
  }

  result.successful = true;
  return result;
}




}  // namespace orunav2_smac_planner


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orunav2_smac_planner::SmacPlannerLattice, nav2_core::GlobalPlanner)

