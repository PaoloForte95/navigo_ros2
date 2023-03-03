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

#ifndef ORUNAV2_SELECTOR__SENSORS__CAMERA_HPP_
#define ORUNAV2_SELECTOR__SENSORS__CAMERA_HPP_

#include <memory>
#include <string>
#include <vector>


#include <sensor_msgs/msg/image.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <ecceleron/navigation/mpc/articulated.h>
#include "geometry_msgs/msg/point.hpp"


#include "orunav2_selector/sensors/source.hpp"



namespace orunav2_selector
{

/**
 * @brief Implementation for laser scanner source
 */
class Image : public orunav2_selector::Source
{
public:
  /**
   * @brief Scan constructor
   * @param node Selector node pointer
   * @param source_name Name of data source
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID. The output data will be transformed into this frame.
   * @param global_frame_id Global frame ID for correct transform calculation
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered valid
   */
  Image(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout);
  /**
   * @brief Scan destructor
   */
  ~Image();

  /**
   * @brief Data source configuration routine. Obtains ROS-parameters
   * and creates laser scanner subscriber.
   */
  void configure();

  /**
   * @brief Adds latest data from laser scanner to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   */
  void getData(
    const rclcpp::Time & curr_time,
    std::vector<geometry_msgs::msg::Point> & data) const;

  /**
   * @brief Adds latest data from laser scanner to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   */
  void getData(
    const rclcpp::Time & curr_time,
    std::vector<double> & data) const;

protected:
  /**
   * @brief Laser scanner data callback
   * @param msg Shared pointer to Image message
   */
  void dataCallback(sensor_msgs::msg::Image msg);


  // ----- Variables -----

  /// @brief Laser scanner data subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr data_sub_;
  
  
  /// @brief Latest data obtained from laser scanner
  sensor_msgs::msg::Image data_;

  //Obstacles
  double max_distance_range_;
  double min_distance_range_;
};  // class Image

}  // namespace orunav2_selector

#endif  // ORUNAV2_SELECTOR__SENSORS__CAMERA_HPP_
