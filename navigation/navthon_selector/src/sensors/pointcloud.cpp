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

#include "navthon_selector/sensors/pointcloud.hpp"

#include <functional>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav2_util/node_utils.hpp"

namespace navthon_selector
{

PointCloud::PointCloud(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout)
: navthon_selector::Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout),
  data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating PointCloud", source_name_.c_str());
}

PointCloud::~PointCloud()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying PointCloud", source_name_.c_str());
  data_sub_.reset();
}

void PointCloud::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

    nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".topic",rclcpp::ParameterValue("/scan/points"));  // Set deafult topic for camera
    source_topic = node->get_parameter(source_name_ + ".topic").as_string();
    nav2_util::declare_parameter_if_not_declared(
        node, source_name_ + ".min_height", rclcpp::ParameterValue(0.05));
    min_height_ = node->get_parameter(source_name_ + ".min_height").as_double();
    nav2_util::declare_parameter_if_not_declared(
        node, source_name_ + ".max_height", rclcpp::ParameterValue(0.5));
    max_height_ = node->get_parameter(source_name_ + ".max_height").as_double();

    rclcpp::QoS pointcloud_qos = rclcpp::SensorDataQoS();  // set to default
    data_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        source_topic, pointcloud_qos,
        std::bind(&PointCloud::dataCallback, this, std::placeholders::_1));
}


void PointCloud::getData(const rclcpp::Time & curr_time, std::vector<geometry_msgs::msg::Point> & data) const
{
    // Ignore data from the source if it is not being published yet or
    // not published for a long time
    if (data_ == nullptr) {
        return;
    }

    tf2::Transform tf_transform;
    // Obtaining the transform to get data from source frame to base frame without time shift
    // considered. Less accurate but much more faster option not dependent on state estimation
    // frames.
    if (
      !nav2_util::getTransform(
        data_->header.frame_id, base_frame_id_,
        transform_tolerance_, tf_buffer_, tf_transform))
    {
      return;
    }
  

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*data_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*data_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*data_, "z");

  // Refill data array with PointCloud points in base frame
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Transform point coordinates from source frame -> to base frame
    tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    if ((p_v3_b.z() >= min_height_ && p_v3_b.z() <= max_height_) and 
         ((p_v3_b.x() >= 0.3 or p_v3_b.x() <= -0.3) and 
         (p_v3_b.y() >= 0.3 or p_v3_b.y() <= -0.3))) {
        geometry_msgs::msg::Point ob_point;
        ob_point.x = p_v3_b.x();
        ob_point.y =  p_v3_b.y();
        data.push_back(ob_point);
    
    }
  }
  RCLCPP_WARN(logger_,"Number of all obstacles: %d", data.size());
}

void PointCloud::getData(const rclcpp::Time & curr_time, std::vector<double> & data) const
{
    //DO nothing
}


void PointCloud::dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor
