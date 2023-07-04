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

#include "navthon_selector/sensors/source.hpp"

#include <exception>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

namespace navthon_selector
{

Source::Source(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout)
: node_(node), source_name_(source_name), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), global_frame_id_(global_frame_id),
  transform_tolerance_(transform_tolerance), source_timeout_(source_timeout)
{
}

Source::~Source()
{
}


bool Source::getTransform(
  const std::string & source_frame_id,
  const rclcpp::Time & source_time,
  const rclcpp::Time & curr_time,
  tf2::Transform & tf2_transform) const
{
  geometry_msgs::msg::TransformStamped transform;
  tf2_transform.setIdentity();  // initialize by identical transform

  try {
    // Obtaining the transform to get data from source to base frame.
    // This also considers the time shift between source and base.
    transform = tf_buffer_->lookupTransform(
      base_frame_id_, curr_time,
      source_frame_id, source_time,
      global_frame_id_, transform_tolerance_);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Failed to get \"%s\"->\"%s\" frame transform: %s",
      source_name_.c_str(), source_frame_id.c_str(), base_frame_id_.c_str(), e.what());
    return false;
  }

  // Convert TransformStamped to TF2 transform
  tf2::fromMsg(transform.transform, tf2_transform);
  return true;
}

}  // namespace nav2_collision_monitor
