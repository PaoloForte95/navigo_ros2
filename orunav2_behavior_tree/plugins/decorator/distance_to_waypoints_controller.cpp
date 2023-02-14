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

#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h"


#include "behaviortree_cpp_v3/decorator_node.h"

#include <nav2_costmap_2d/array_parser.hpp>

#include "orunav2_behavior_tree/plugins/decorator/distance_to_waypoints_controller.hpp"

namespace orunav2_behavior_tree
{

DistanceToWaypointsController::DistanceToWaypointsController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  distance_(1.0),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  first_time_(false)
{
  getInput("distance", distance_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  pub_waypoint = node_->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_visualization_marker_array", 10);

  getInput("waypoints", waypoints_string_);
  geometry_msgs::msg::Pose wp;


std::string error = "The waypoint (parameter %s) must be specified as list of lists eg: ""[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.";
auto vertices_front = nav2_costmap_2d::parseVVF(waypoints_string_,error);
for (int i = 0; i < vertices_front.size(); ++i)
    {
       // Make sure each element of the list is an array of size 2. (x and y coordinates)
       std::vector<float> point = vertices_front[ i ];
      if (point.size() != 2)
      {
       RCLCPP_FATAL(node_->get_logger(), "%s/n",error);
       throw std::runtime_error(error);
      }
      //Getting the points and save them
      wp.position.x = static_cast<double>(point[ 0 ]);
      wp.position.y  = static_cast<double>(point[ 1 ]);
      RCLCPP_INFO(node_->get_logger(), "Adding waypoint: %f %f", wp.position.x, wp.position.y );
      waypoints_poses_.push_back(wp);

    }
//Publish the waypoint 

double scale = 1.0;
for (size_t i = 0; i < waypoints_poses_.size(); i++) {
  visualization_msgs::msg::Marker m;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.ns = "waypoint" + std::to_string(i);
  m.id = i;
  m.header.frame_id = global_frame_;
  m.pose.position = waypoints_poses_[i].position;
  m.pose.position.z = 10;
  m.color.r = 1;
  m.color.g = 0;
  m.color.b = 0;
  m.color.a = 0.7;

  m.scale.x = 1.0 * scale;
  m.scale.y = 1.0 * scale;
  m.scale.z = 1.0 * scale;

  ma.markers.push_back(m);

  //Create a cylinder to show the area nearby the point
  m.type = visualization_msgs::msg::Marker::CYLINDER;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.ns = "waypoint_circle" + std::to_string(i);
  m.id = i + waypoints_poses_.size();
  m.color.r = 1;
  m.color.g = 0;
  m.color.b = 0;
  m.color.a = 0.7;
  m.pose.position = waypoints_poses_[i].position;
  m.pose.position.z = 10;

  m.scale.x = 1.0 * distance_;
  m.scale.y = 1.0 * distance_;
  m.scale.z = 0.1;

  ma.markers.push_back(m);

  
}

  if(!ma.markers.empty()){
      pub_waypoint->publish(ma);
 }


}

inline BT::NodeStatus DistanceToWaypointsController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting position since we're starting a new iteration of
    // the distance controller (moving from IDLE to RUNNING)
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // Determine distance travelled since we've started this iteration
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }


  // Get euclidean distance from all waypoints
  for(auto waypoint : waypoints_poses_){
    auto distance = nav2_util::geometry_utils::euclidean_distance( waypoint, current_pose.pose);
   
    
    // The child gets ticked the first time through and every time the threshold
    // distance is crossed. In addition, once the child begins to run, it is
    // ticked each time 'til completion
    if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
      distance <= distance_ && previous_closest_waypoint_ != waypoint)
    {
      if(first_time_){
          first_time_ = false;
      }else{
        previous_closest_waypoint_ = waypoint;
        RCLCPP_INFO(node_->get_logger(), "Nearby a waypoint! Entering a new area! Restoring all planners!");
      }

      const BT::NodeStatus child_state = child_node_->executeTick();


      switch (child_state) {
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::SUCCESS:
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
        default:
          return BT::NodeStatus::FAILURE;
      }
    }
  }
  

  return status();
}

}  // namespace orunav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orunav2_behavior_tree::DistanceToWaypointsController>("DistanceToWaypointsController");
}
