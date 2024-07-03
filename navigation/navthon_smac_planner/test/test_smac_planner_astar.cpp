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
// limitations under the License. Reserved.

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "navthon_smac_planner/smac_planner_astar.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

// Simple wrapper to be able to call a private member
class LatticeWrap : public navthon_smac_planner::SmacPlannerAStar
{
public:
  void callDynamicParams(std::vector<rclcpp::Parameter> parameters)
  {
    dynamicParametersCallback(parameters);
  }
};

// SMAC smoke tests for plugin-level issues rather than algorithms
// (covered by more extensively testing in other files)
// System tests in nav2_system_tests will actually plan with this work

TEST(SmacTest, test_smac_planner_astar)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeAStar =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacAStarTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  std::shared_ptr<tf2_ros::Buffer> tf = costmap_ros->getTfBuffer();  
  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 7.0;
  goal.pose.position.y = 0.0;
  goal.pose.orientation.w = 1.0;
  auto planner = std::make_unique<navthon_smac_planner::SmacPlannerAStar>();

  try {
    // Expect to throw due to invalid prims file in param
    planner->configure(nodeAStar, "test", nullptr, costmap_ros);

  } catch (...) {
  }
  planner->activate();

  try {
    planner->createPlan(start, goal);
  } catch (...) {
  }

  planner->deactivate();
  planner->cleanup();

  planner.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  costmap_ros.reset();
  nodeAStar.reset();
}
