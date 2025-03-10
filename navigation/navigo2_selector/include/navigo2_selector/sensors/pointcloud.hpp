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

#ifndef NAVIGO2_SELECTOR__SENSORS__POINTCLOUD_HPP_
#define NAVIGO2_SELECTOR__SENSORS__POINTCLOUD_HPP_

#include <memory>
#include <vector>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_util/robot_utils.hpp"

#include "navigo2_selector/sensors/source.hpp"

namespace navigo2_selector
{

/**
 * @brief Implementation for pointcloud source
 */
class PointCloud : public navigo2_selector::Source
{
public:
  /**
   * @brief PointCloud constructor
   * @param node Collision Monitor node pointer
   * @param source_name Name of data source
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID. The output data will be transformed into this frame.
   * @param global_frame_id Global frame ID for correct transform calculation
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered valid
   * considering the difference between current time and latest source time
   */
  PointCloud(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout);
  /**
   * @brief PointCloud destructor
   */
  ~PointCloud();

  /**
   * @brief Data source configuration routine. Obtains pointcloud related ROS-parameters
   * and creates pointcloud subscriber.
   */
  void configure();

  /**
   * @brief Adds latest data from pointcloud to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   */
  void getData(
    const rclcpp::Time & curr_time,
    std::vector<geometry_msgs::msg::Point> & data) const;

  /**
   * @brief Adds latest data from pointcloud to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   */
  void getData(
    const rclcpp::Time & curr_time,
    std::vector<double> & data) const;

protected:


  /**
   * @brief PointCloud data callback
   * @param msg Shared pointer to PointCloud message
   */
  void dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // ----- Variables -----

  /// @brief PointCloud data subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr data_sub_;

  // Minimum and maximum height of PointCloud projected to 2D space
  double min_height_, max_height_;

  /// @brief Latest data obtained from pointcloud
  sensor_msgs::msg::PointCloud2::ConstSharedPtr data_;
};  // class PointCloud

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POINTCLOUD_HPP_
