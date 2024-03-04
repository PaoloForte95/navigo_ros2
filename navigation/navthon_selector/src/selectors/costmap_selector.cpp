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

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "navthon_selector/selectors/costmap_selector.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/footprint.hpp"

// #define BENCHMARK_TESTING
using nav2_costmap_2d::makeFootprintFromString;
namespace navthon_selector
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
    node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
  node->get_parameter<bool>(name + ".downsample_costmap", _downsample_costmap);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  node->get_parameter<int>(name + ".downsampling_factor", _downsampling_factor);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter<bool>(name + ".allow_unknown", _allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".distance_threshold", rclcpp::ParameterValue(3.0));
  node->get_parameter<double>(name + ".distance_threshold", _distance_threshold);
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".points", rclcpp::ParameterValue(std::string("[]")));
  node->get_parameter<std::string>(name + ".points", _points);
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".max_points", rclcpp::ParameterValue(3));
  _max_points = node->get_parameter(name + ".max_points").as_int();
  nav2_util::declare_parameter_if_not_declared(
        node, name + ".polygon_pub_topic", rclcpp::ParameterValue("polygon_en"));
  polygon_pub_topic = node->get_parameter(name + ".polygon_pub_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".base_frame_id", rclcpp::ParameterValue("base_link"));
  _base_frame_id = node->get_parameter(name + ".base_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".odom_frame_id", rclcpp::ParameterValue("world"));
  _odom_frame_id = node->get_parameter(name + ".odom_frame_id").as_string();

  nav2_costmap_2d::makeFootprintFromString(_points,_poly_points);
  // Check for points format correctness
  if (_poly_points.size() <= 3 || _poly_points.size() % 2 != 0) {
    RCLCPP_ERROR(
      _logger,
      "Polygon has incorrect points description");
  }

  // Initialize costmap downsampler
  if (_downsample_costmap && _downsampling_factor > 1) {
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler = std::make_unique<nav2_smac_planner::CostmapDownsampler>();
    _costmap_downsampler->on_configure(
      node, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(polygon_pub_topic, rclcpp::SystemDefaultsQoS());

  nav2_util::declare_parameter_if_not_declared(node, name + ".observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
  std::vector<std::string> source_names = node->get_parameter(name + ".observation_sources").as_string_array();
  tf2::Duration transform_tolerance = tf2::durationFromSec(0.5);
  rclcpp::Duration source_timeout = rclcpp::Duration::from_seconds(6.0);
  for (std::string source_name : source_names) {
    nav2_util::declare_parameter_if_not_declared(node, name + "." + source_name + ".type",rclcpp::ParameterValue(""));  // Laser scanner by default
    const std::string source_type = node->get_parameter(name + "." + source_name + ".type").as_string();
    RCLCPP_INFO(node->get_logger(),"Adding source of type: %s", source_type.c_str());
    
    if (source_type == "scan") {
      std::shared_ptr<navthon_selector::Scan> s = std::make_shared<navthon_selector::Scan>(node, name + "." + source_name, costmap_ros->getTfBuffer(), _base_frame_id, _odom_frame_id, transform_tolerance, source_timeout);
      s->configure();
      _sources.push_back(s);

    } else if (source_type == "image") {
      std::shared_ptr<navthon_selector::Image> p = std::make_shared<navthon_selector::Image>(node, name + "." + source_name, costmap_ros->getTfBuffer(), _base_frame_id, _odom_frame_id, transform_tolerance, source_timeout);
      p->configure();
      _sources.push_back(p);
    }else if (source_type == "pointcloud") {
      std::shared_ptr<navthon_selector::PointCloud> p = std::make_shared<navthon_selector::PointCloud>(node, name + "." + source_name, costmap_ros->getTfBuffer(), _base_frame_id, _odom_frame_id, transform_tolerance, source_timeout);
      p->configure();
      _sources.push_back(p);
    }  else {  // Error if something else
      RCLCPP_ERROR(node->get_logger(),"[%s]: Unknown source type: %s", source_name.c_str(), source_type.c_str());
    }
  }

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type CostmapSelector with "
    "distance_threshold %.2f, and %s.",
    _name.c_str(),_distance_threshold,
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
  polygon_pub_->on_activate();
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
  polygon_pub_->on_deactivate();
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
  polygon_pub_.reset();
}

std::string CostmapSelector::selectGlobalPlanner(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<std::string> & planner_ids)
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
      RCLCPP_WARN(_logger, "Ouch!");
      return selected_planner;
    }
    return selected_planner;
  }
  //RCLCPP_INFO(_logger, " CostMap cost at starting position %f", costmap->getCost(mx_start, my_start));

  //Get the current robot position
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  RCLCPP_INFO(_logger, "Robot current position: (%f, %f)", start_x, start_y);

  // Fill collision_points array from different data sources
  auto node = _node.lock();
  rclcpp::Time curr_time =  node->get_clock()->now();

  //Scan, PointCloud and Range
  std::vector<geometry_msgs::msg::Point> collision_points;
  for (std::shared_ptr<navthon_selector::Source> source : _sources) {
    source->getData(curr_time, collision_points);
  }

  //Image
  std::vector<double> colors;
    for (std::shared_ptr<navthon_selector::Source> source : _sources) {
    source->getData(curr_time, colors);
  }


  RCLCPP_INFO(_logger, "Collision Points %d", collision_points.size());
  std::vector<int> insidePoints = getPointsInside(collision_points);
  RCLCPP_INFO(_logger, "Point inside left polygon: %d and right polygon: %d", insidePoints[0], insidePoints[1]);
  if (insidePoints[0] >= _max_points && insidePoints[1] >= _max_points ){
    RCLCPP_INFO(_logger, "Obstacles on both sides! Entering a corridor!");
    selected_planner = "GridBased";
  }
  else if (insidePoints[0] > _max_points){
    RCLCPP_INFO(_logger, "Obstacles on left side!");
    selected_planner = "GridBased";
  
  }
  else if(insidePoints[1] > _max_points){
    RCLCPP_INFO(_logger, "Obstacles on right side!");
    selected_planner = "GridBased";
  }
  else {
    RCLCPP_INFO(_logger, " No obstacles detected in the polygons. Open space ");
    selected_planner = "GridBased";
  }

  //After selected the planner let's check if it failed before to compute the path
  auto itr = std::find(planner_ids.begin(), planner_ids.end(), selected_planner);
  if (itr == planner_ids.end() || planner_ids.empty()){
     std::string default_planner = "GridBased";
     RCLCPP_INFO(_logger, "Selected planning algorithm %s failed to compute a plan before. Changing to %s ", selected_planner.c_str(), default_planner.c_str() );
     selected_planner = default_planner;
  }

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  RCLCPP_INFO(_logger, "Best global planner is: %s ", selected_planner.c_str());
  publishPolygon();
  return selected_planner;
}



void CostmapSelector::publishPolygon() const {
  geometry_msgs::msg::Polygon polygon_;
   auto node = _node.lock();

  for (const geometry_msgs::msg::Point & p : _poly_points) {
    geometry_msgs::msg::Point32 p_s;
    p_s.x = p.x;
    p_s.y = p.y;
    // p_s.z will remain 0.0
    polygon_.points.push_back(p_s);
  }
  std::unique_ptr<geometry_msgs::msg::PolygonStamped> poly_s = std::make_unique<geometry_msgs::msg::PolygonStamped>();
 
  poly_s->header.stamp =  node->get_clock()->now();
  poly_s->header.frame_id = _base_frame_id;
  poly_s->polygon = polygon_;
  polygon_pub_->publish(std::move(poly_s));
}


std::vector<int> CostmapSelector::getPointsInside(const std::vector<geometry_msgs::msg::Point> & points) const
{
  int num_l = 0, num_r = 0;
  std::vector<int> nums;
  std::vector<geometry_msgs::msg::Point> _poly_left_points,_poly_right_points;
  geometry_msgs::msg::Point middle1, middle2;
  middle1.y = (_poly_points[1].y + _poly_points[0].y)/2;
  middle1.x = _poly_points[0].x;
  middle2.y = (_poly_points[2].y + _poly_points[3].y)/2;
  middle2.x = _poly_points[3].x;
  //Add th point to left and right polygon
  _poly_left_points.push_back(middle1);
  _poly_left_points.push_back(_poly_points[1]);
  _poly_left_points.push_back(_poly_points[2]);
  _poly_left_points.push_back(middle2);

  _poly_right_points.push_back(_poly_points[0]);
  _poly_right_points.push_back(middle1);
  _poly_right_points.push_back(middle2);
  _poly_right_points.push_back(_poly_points[3]);
  for (const geometry_msgs::msg::Point & point : points) {
    if (isPointInside(point, _poly_left_points )) {
      num_l++;
    }
    if (isPointInside(point, _poly_right_points )) {
      num_r++;
    }
  }
  nums.push_back(num_l);
  nums.push_back(num_r);
  return nums;
}


inline bool CostmapSelector::isPointInside(const geometry_msgs::msg::Point & point, std::vector<geometry_msgs::msg::Point> poly_points ) const
{
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
  // Communications of the ACM 5.8 (1962): 434.
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int poly_size = poly_points.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the first
  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates.
    // One of the condition should contain equality in order to exclude the edges
    // parallel to X+ ray.
    if ((point.y <= poly_points[i].y) == (point.y > poly_points[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      const double x_inter = poly_points[i].x +
        (point.y - poly_points[i].y) * (poly_points[j].x - poly_points[i].x) /
        (poly_points[j].y - poly_points[i].y);
      // If intersection with checked edge is greater than point.x coordinate, inverting the result
      if (x_inter > point.x) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}


rcl_interfaces::msg::SetParametersResult
CostmapSelector::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  bool reinit_downsampler = false;


  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == _name + ".distance_threshold") {
        _distance_threshold = static_cast<float>(parameter.as_double());
      }

    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == _name + ".downsample_costmap") {
        reinit_downsampler = true;
        _downsample_costmap = parameter.as_bool();
      } else if (name == _name + ".allow_unknown") {
        _allow_unknown = parameter.as_bool();
      }
    }
    else if (type == ParameterType::PARAMETER_INTEGER){
      if (name == _name + ".downsampling_factor") {
        _downsampling_factor = parameter.as_bool();
         reinit_downsampler = true;
      } else if (name == _name + ".max_points") {
        _max_points = parameter.as_bool();
      } 
    }
    else if (type == ParameterType::PARAMETER_STRING){
      if (name == _name + ".points") {
        _points = parameter.as_string();
      } else if (name == _name + ".polygon_pub_topic") {
        polygon_pub_topic = parameter.as_string();
      } else if (name == _name + ".base_frame_id") {
        _base_frame_id = parameter.as_string();
      } else if (name == _name + ".odom_frame_id") {
        _odom_frame_id = parameter.as_string();
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


}  // namespace navthon_selector

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(navthon_selector::CostmapSelector, navthon_core::GlobalPlannerSelector)
