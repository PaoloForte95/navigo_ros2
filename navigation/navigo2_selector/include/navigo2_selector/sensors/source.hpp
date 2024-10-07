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

#ifndef NAVIGO2_SELECTOR__SOURCE_HPP_
#define NAVIGO2_SELECTOR__SOURCE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"

#include "nav2_util/lifecycle_node.hpp"

#include "geometry_msgs/msg/point.hpp"

namespace navigo2_selector
{

/**
 * @brief Basic data source class
 */
class Source
{
public:
  /**
   * @brief Source constructor
   * @param node Selector node pointer
   * @param source_name Name of data source
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID. The output data will be transformed into this frame.
   * @param global_frame_id Global frame ID for correct transform calculation
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered valid
   */
  Source(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout);
  /**
   * @brief Source destructor
   */
  virtual ~Source();

  /**
   * @brief Adds latest data from source to the data array.
   * Empty virtual method intended to be used in child implementations.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   */
  virtual void getData(
    const rclcpp::Time & curr_time,
    std::vector<geometry_msgs::msg::Point> & data) const = 0;


   /**
   * @brief Adds latest data from source to the data array.
   * Empty virtual method intended to be used in child implementations.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   */
  virtual void getData(
    const rclcpp::Time & curr_time,
    std::vector<double> & data) const = 0;
  

protected:

  /**
   * @brief Obtains a transform from source_frame_id at source_time ->
   * to base_frame_id_ at curr_time time
   * @param source_frame_id Source frame ID to convert data from
   * @param source_time Source timestamp to convert data from
   * @param curr_time Current node time to interpolate data to
   * @param tf_transform Output source->base transform
   * @return True if got correct transform, otherwise false
   */
  bool getTransform(
    const std::string & source_frame_id,
    const rclcpp::Time & source_time,
    const rclcpp::Time & curr_time,
    tf2::Transform & tf_transform) const;

  // ----- Variables -----

  /// @brief Global Planner Selector node
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  /// @brief Global Planner Selector node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("global_planner_selector")};

  // Basic parameters
  /// @brief Name of data source
  std::string source_name_;

  // Global variables
  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief Robot base frame ID
  std::string base_frame_id_;
  /// @brief Global frame ID for correct transform calculation
  std::string global_frame_id_;
  /// @brief Transform tolerance
  tf2::Duration transform_tolerance_;
  /// @brief Maximum time interval in which data is considered valid
  rclcpp::Duration source_timeout_;
};  // class Source

}  // namespace navigo2_selector

#endif  // NAVIGO2_SELECTOR__SOURCE_HPP_
