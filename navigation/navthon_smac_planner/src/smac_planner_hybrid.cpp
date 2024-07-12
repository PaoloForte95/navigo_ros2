#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include <navigatio/planner/car_planner.h>
#include <navthon_smac_planner/smac_planner_hybrid.hpp>

namespace navthon_smac_planner

{

using namespace std::chrono;  // NOLINT
using namespace navigatio;
using rcl_interfaces::msg::ParameterType;

MotionModel fromString(const std::string & n)
{
  if (n == "2D") {
    return MotionModel::TWOD;
  } else if (n == "DUBIN") {
    return MotionModel::DUBIN;
  } else if (n == "REEDS_SHEPP") {
    return MotionModel::REEDS_SHEPP;
  } else {
    return MotionModel::UNKNOWN;
  }
}

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
  {
  	tf2::Quaternion q;
  	q.setRPY(0, 0, yaw);
  	return tf2::toMsg(q);
  }	

  nav_msgs::msg::Path createPathMsgFromPathInterface(const Path &path)
  {
    nav_msgs::msg::Path p;
    for (size_t i = 0; i < path.size(); i++)
      {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.orientation = createQuaternionMsgFromYaw(path[i].theta);
        ps.pose.position.x = path[i].x;
        ps.pose.position.y = path[i].y;
        ps.pose.position.z = 0.;
        p.poses.push_back(ps);
      }
    return p;
  }



SmacPlannerHybrid::SmacPlannerHybrid(): 
  costmap_(nullptr), planner_(nullptr)
{
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  RCLCPP_INFO( logger_, "Destroying SmacPlannerHybrid", name_.c_str());
}

void SmacPlannerHybrid::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  double max_planning_time = 10, minimum_turning_radius;
  bool allow_unknown;
  std::string primitives, primitives_directory, model;

  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  global_frame_ = costmap_ros->getGlobalFrameID();
  costmap_ = costmap_ros->getCostmap();
  name_ = name;
  raw_plan_publisher_ = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  RCLCPP_INFO(logger_, "Configuring SmacPlannerHybrid...");
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_time", rclcpp::ParameterValue(10.0));
  node->get_parameter(name + ".max_planning_time", max_planning_time);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_turning_radius", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".minimum_turning_radius", minimum_turning_radius);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".model", rclcpp::ParameterValue("2D"));
  node->get_parameter(name + ".model", model);


  //Create the planner
  navigatio::SearchParams search_params;
  navigatio::PlannerParams planner_params;
  search_params.minimum_turning_radius = minimum_turning_radius;

  //Set Planning Parameters
  planner_params.max_planning_time = max_planning_time;
  planner_params.allow_unknown = allow_unknown;
  planner_ = new CarPlanner("CarPlanner", fromString(model), search_params, planner_params, PlanningAlgorithm::RRTstar);
  
  

  robot_footprint_ = Footprint(Figure::Polygon);
  std::vector<Eigen::Vector2d> points;
  for(auto point : costmap_ros->getRobotFootprint()){
    points.push_back(Eigen::Vector2d(point.x,point.y));
  }
  robot_footprint_.makeFootprint(points);

  nav2_util::declare_parameter_if_not_declared(node, name + ".front_frame_id", rclcpp::ParameterValue("front_link_axis"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".rear_frame_id", rclcpp::ParameterValue("rear_link_axis"));
  front_frame_id_ = node->get_parameter(name + ".front_frame_id").as_string();
  rear_frame_id_ = node->get_parameter(name + ".rear_frame_id").as_string();
  RCLCPP_INFO( logger_, "Configured SmacPlannerHybrid! ");
}

void SmacPlannerHybrid::activate()
{
  RCLCPP_INFO(
    logger_, "Activating plugin %s of type SmacPlannerHybrid",
    name_.c_str());
  raw_plan_publisher_->on_activate();
  auto node = node_.lock();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SmacPlannerHybrid::dynamicParametersCallback, this, std::placeholders::_1));
}

void SmacPlannerHybrid::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating plugin %s of type SmacPlannerHybrid",
    name_.c_str());
  raw_plan_publisher_->on_deactivate();
  dyn_params_handler_.reset();
}

void SmacPlannerHybrid::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type SmacPlannerHybrid",
    name_.c_str());
  raw_plan_publisher_.reset();
}

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_INFO(logger_, "Computing Global Plan with Lattice-Based Algorithm");
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  //Information about start and goal
  double steering_start = 0.0;
  double steering_goal = 0.0;

  int xcells = costmap_->getSizeInCellsX();
  int ycells = costmap_->getSizeInCellsY();
  Eigen::MatrixXd costmap_matrix(xcells, ycells);
  for (unsigned int i = 0; i < xcells; i++)
  {
    for (unsigned int j = 0; j < ycells; j++)
    {
      costmap_matrix(i,j) = costmap_->getCost(i,j);
    }
  }
  auto map = std::make_unique<CostMap>(costmap_matrix, costmap_->getResolution(), costmap_->getOriginX(), costmap_->getOriginY());
  std::unique_ptr<GridCollisionChecker> checker = std::make_unique<GridCollisionChecker>(map.get());
  checker->setFootprint(robot_footprint_);
  
  planner_->setCollisionChecker(checker.get());


  RCLCPP_INFO(logger_, "[SmacPlannerHybrid] - start : [%f,%f,%f](%f)", start.pose.position.x, start.pose.position.y,tf2::getYaw(start.pose.orientation), steering_start);
  RCLCPP_INFO(logger_, "[SmacPlannerHybrid] - goal :  [%f,%f,%f](%f)", goal.pose.position.x, goal.pose.position.y,tf2::getYaw(goal.pose.orientation), steering_goal);
    

  double map_offset_x = costmap_->getOriginX();
  double map_offset_y = costmap_->getOriginY();
  double start_orientation = tf2::getYaw(start.pose.orientation);
  double goal_orientation = tf2::getYaw(goal.pose.orientation);
  Pose start_pose(start.pose.position.x, start.pose.position.y, start_orientation);
  Pose goal_pose(goal.pose.position.x,  goal.pose.position.y, goal_orientation);
  nav_msgs::msg::Path plan;
  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;  
 
  Path path = planner_->computePath(start_pose,goal_pose);
  bool solution_found = true;
        
    if (path.size() == 0)
      solution_found = false;
  plan = createPathMsgFromPathInterface(path);
  plan.header.frame_id = global_frame_;
  // Publish raw path for debug
  if (raw_plan_publisher_->get_subscription_count() > 0) {
    raw_plan_publisher_->publish(plan);
  }

  return plan;
}


rcl_interfaces::msg::SetParametersResult
SmacPlannerHybrid::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  double max_planning_time, minimum_turning_radius;
  bool allow_unknown;
  std::string primitives, primitives_directory;
  bool reinit_lattice_planner = false;
  bool reinit_smoother = false;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".max_planning_time") {
        reinit_lattice_planner = true;
        max_planning_time = parameter.as_double();
      } else if (name == name_ + ".minimum_turning_radius") {
        reinit_lattice_planner = true;
        minimum_turning_radius = parameter.as_double();
      }

    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".allow_unknown") {
        reinit_lattice_planner = true;
        allow_unknown = parameter.as_bool();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == name_ + ".primitives_directory") {
        reinit_lattice_planner = true;
        primitives_directory = parameter.as_string();
      }
      else if (name == name_ + ".primitives") {
        reinit_lattice_planner = true;
        primitives = parameter.as_string();
      }
    }
  }

  // Re-init the lattice planner
  if (reinit_lattice_planner) {
      LatticeMetadata search_params;
      PlannerParams planner_params;
      search_params.minimum_turning_radius = minimum_turning_radius;

      //Set Planning Parameters
      planner_params.max_planning_time = max_planning_time;
      planner_params.allow_unknown = allow_unknown;

    planner_ = new CarPlanner("CarPlanner", MotionModel::REEDS_SHEPP, search_params, planner_params, PlanningAlgorithm::RRTstar);

    int xcells = costmap_->getSizeInCellsX();
    int ycells = costmap_->getSizeInCellsY();
    Eigen::MatrixXd costmap_matrix(ycells, xcells);
    unsigned int k = 0;
    for (unsigned int i = 0; i < ycells; i++)
    {
      for (unsigned int j = 0; j < xcells; j++)
      {
        costmap_matrix(i,j) = costmap_->getCost(k);
        k++;
      }
    }
    auto map = std::make_unique<CostMap>(costmap_matrix, costmap_->getResolution(), costmap_->getOriginX(),costmap_->getOriginY());

   
  std::unique_ptr<GridCollisionChecker> checker = std::make_unique<GridCollisionChecker>(map.get());
  checker->setFootprint(robot_footprint_);
  
  planner_->setCollisionChecker(checker.get());
    RCLCPP_INFO(logger_, "Restarting planner with new parameters");
  }

  result.successful = true;
  return result;
}

}  


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(navthon_smac_planner::SmacPlannerHybrid, nav2_core::GlobalPlanner)

