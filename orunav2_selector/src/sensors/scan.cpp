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

#include <cmath>
#include <functional>

#include "orunav2_selector/sensors/scan.hpp"
#include "nav2_util/node_utils.hpp"


namespace orunav2_selector
{

Scan::Scan(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Scan", source_name_.c_str());
}

Scan::~Scan()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Scan", source_name_.c_str());
  data_sub_.reset();
}

void Scan::configure()
{
  auto node = node_.lock();
  logger_ = node->get_logger();
  
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;
  
  // Get common parameters
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".topic",rclcpp::ParameterValue("scan"));  // Set deafult topic for laser scanner
  source_topic = node->get_parameter(source_name_ + ".topic").as_string();
  
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".max_obstacle_range", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".min_distance_between_obstacles", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".max_nb_obstacles", rclcpp::ParameterValue(8));

  max_obstacle_range_ = node->get_parameter(source_name_ + ".max_obstacle_range").as_double();
  min_distance_between_obstacles_ = node->get_parameter(source_name_ + ".min_distance_between_obstacles").as_double();
  max_nb_obstacles_ = node->get_parameter(source_name_ + ".max_nb_obstacles").as_int();

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default

  data_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(source_topic, scan_qos,std::bind(&Scan::dataCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "[%s]: Scan subscribing to topic: %s", source_name_.c_str(), source_topic.c_str() );
}

/*
void Scan::getData(
  const rclcpp::Time & curr_time,
  std::vector<nav2_collision_monitor::Point> & data) const
{
  // Ignore data from the source if it is not being published yet or
  // not being published for a long time
  if (data_ == nullptr) {
    return;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return;
  }

  // Obtaining the transform to get data from source frame and time where it was received
  // to the base frame and current time
  tf2::Transform tf_transform;
  if (!getTransform(data_->header.frame_id, data_->header.stamp, curr_time, tf_transform)) {
    return;
  }

  // Calculate poses and refill data array
  float angle = data_->angle_min;
  for (size_t i = 0; i < data_->ranges.size(); i++) {
    if (data_->ranges[i] >= data_->range_min && data_->ranges[i] <= data_->range_max) {
      // Transform point coordinates from source frame -> to base frame
      tf2::Vector3 p_v3_s(
        data_->ranges[i] * std::cos(angle),
        data_->ranges[i] * std::sin(angle),
        0.0);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

      // Refill data array
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
    angle += data_->angle_increment;
  }
*/

void Scan::getData(
  const rclcpp::Time & curr_time,
  std::vector<nav2_collision_monitor::Point> & data) const
{
    sensor_msgs::msg::PointCloud2 cloud;
    laser_geometry::LaserProjection laser_projection;
    try
    {
    laser_projection.transformLaserScanToPointCloud(base_frame_id_, data_, cloud, *tf_buffer_, max_obstacle_range_);

    // To avoid that to many point obstacles are generated, subsample the cloud while preserving important points.

    // For this simple operation, just utilize PointCloud message as it is easier to access the elements
    sensor_msgs::msg::PointCloud tmp;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud, tmp);
    ecl::navigation::PointDistancesSelected pds = convertPointCloudMsgToPointDistancesSelected(tmp);

    ecl::navigation::pointDistancesSelectFirstAndLastSelectFilter(pds);
    ecl::navigation::pointDistancesLocalMinimaSelectFilter(pds, 5);
    pds.assignDistanceBetweenSelected(min_distance_between_obstacles_);
    ecl::navigation::PointDistances pd = pds.getSelected();
    ecl::navigation::PointDistances pd_N = pointDistancesGetNClosest(pd, max_nb_obstacles_);
    ecl::navigation::Point2dObstacles obst = convertPointDistanceContainerInterfaceToPoint2dObstacles(pd_N);

    tmp.points.clear();
    for (auto o : obst)
  {
    geometry_msgs::msg::Point32 p;
    p.x = o.x();
    p.y = o.y();
    p.z = 0.;
    tmp.points.push_back(p);
    nav2_collision_monitor::Point ob_point;
    ob_point.x = o.x();
    ob_point.y = o.y();
    data.push_back(ob_point);
  }
  RCLCPP_WARN(logger_,"Number of all obstacles: %d", data.size());
  sensor_msgs::convertPointCloudToPointCloud2(tmp, cloud);
  }catch (tf2::TransformException &ex)
        {
        RCLCPP_WARN(logger_,"%s", ex.what());
      } 

}

void Scan::dataCallback(sensor_msgs::msg::LaserScan msg)
{
  data_ = msg;
}


ecl::navigation::PointDistancesSelected Scan::convertPointCloudMsgToPointDistancesSelected(const sensor_msgs::msg::PointCloud &cloud) const
  {
    ecl::navigation::PointDistancesSelected ret;
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
      ecl::navigation::PointDistanceSelected p(Eigen::Vector2d(cloud.points[i].x, cloud.points[i].y));
      ret.push_back(p);
    }
    return ret;
  }

}  // namespace orunav2_selector
