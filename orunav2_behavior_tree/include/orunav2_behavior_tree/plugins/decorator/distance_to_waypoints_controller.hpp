// Copyright (c) 2023 paolo Forte
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

#ifndef ORUNAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__DISTANCE_TO_WAYPOINTS_CONTROLLER_HPP_
#define ORUNAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__DISTANCE_TO_WAYPOINTS_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"


namespace orunav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child every time the robot
 * is within a certain distance from one of the defined waypoints
 */
class DistanceToWaypointsController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for orunav2_behavior_tree::DistanceToWaypointsController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  DistanceToWaypointsController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Distance"),
      BT::InputPort<std::string>("waypoints", std::string(""), "The set of waypoints"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
  std::string waypoints_string_;
  std::vector<geometry_msgs::msg::Pose> waypoints_poses_;
  geometry_msgs::msg::Pose previous_closest_waypoint_;
  double distance_;
  std::string global_frame_;
  std::string robot_base_frame_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_waypoint;
  visualization_msgs::msg::MarkerArray ma;
  
  bool first_time_;
};

}  // namespace orunav2_behavior_tree

#endif  // ORUNAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__DISTANCE_CONTROLLER_HPP_
