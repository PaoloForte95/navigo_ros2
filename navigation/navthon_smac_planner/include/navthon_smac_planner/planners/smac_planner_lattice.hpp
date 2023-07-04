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

#ifndef NAVTHON_SMAC_PLANNER__SMAC_PLANNER_LATTICE_HPP_
#define NAVTHON_SMAC_PLANNER__SMAC_PLANNER_LATTICE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "tf2/utils.h"

//nav2
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "navthon_smac_planner/smoother.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

//msg
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

//navthon
#include "navthon_smac_planner/types.hpp"
#include "navthon_smac_planner/constants.hpp"
#include "navthon_smac_planner/collision_detector.hpp"
#include "navthon_smac_planner/model/lhd_model.hpp"
#include "navthon_smac_planner/path_finder.hpp"
#include "navthon_smac_planner/world_occupancy_map.hpp"



namespace navthon_smac_planner
{

class SmacPlannerLattice : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlannerLattice();

  /**
   * @brief destructor
   */
  ~SmacPlannerLattice();

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

    // Uses tf to find out the joint angles
  bool getArticulatedJointAngle(double &angle) const;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav2_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;



    /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

protected:
   

  std::shared_ptr<tf2_ros::Buffer> _tf; //!< pointer to tf buffer
  std::unique_ptr<nav2_smac_planner::Smoother> _smoother;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("SmacPlannerLattice")};
  nav2_costmap_2d::Costmap2D * _costmap;
  navthon_smac_planner::MotionModel _motion_model;
  nav2_smac_planner::LatticeMetadata _metadata;
  std::string _global_frame, _name;
  navthon_smac_planner::SearchInfo _search_info;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  bool _use_final_approach_orientation;
  float _tolerance;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  double _max_planning_time;
  double _lookup_table_size;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;
  navthon_smac_planner::LHDModel* _model;
  navthon_smac_planner::PathFinder* _path_finder;
  std::string model;
  bool _save_paths;
  double _min_incr_path_dist;
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
  std::string _front_frame_id;
  std::string _rear_frame_id;
};



}  // namespace navthon_smac_planner




#endif  // NAVTHON_SMAC_PLANNER__SMAC_PLANNER_LATTICE_HPP_

