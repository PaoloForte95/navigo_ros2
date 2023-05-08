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
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "orunav2_selector/sensors/image.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/lifecycle_node.hpp"
#include "orunav2_selector/global_planner_selector_server.hpp"


class TestNode : public nav2_util::LifecycleNode
{
public:
  TestNode()
  : nav2_util::LifecycleNode("test_node")
  {
  }

  ~TestNode()
  {
  }
  

};


TEST(SourceImageTest, test_source_image)
{  
    
  auto test_node_ = std::make_shared<TestNode>();
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(test_node_->get_clock());
  auto name ="";
  std::shared_ptr<orunav2_selector::Image> p = std::make_shared<orunav2_selector::Image>(test_node_, name , tf_buffer, "base_link", "odom", tf2::durationFromSec(0.5), rclcpp::Duration::from_seconds(6.0));
  p->configure();
  rclcpp::Time start_time = test_node_->now();
  auto timeout = std::chrono::seconds(2);
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rclcpp::Time curr_time =  test_node_->get_clock()->now();
  std::vector<double> colors;
  p->getData(curr_time,colors );
  rclcpp::spin_some(test_node_->get_node_base_interface());

}


TEST(SourceImageTest2, test_source_image2)
{  
    
  auto test_node_ = std::make_shared<orunav2_selector::PlannerSelectorServer>();
  
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(test_node_->get_clock());
  auto name ="";
  std::shared_ptr<orunav2_selector::Image> p = std::make_shared<orunav2_selector::Image>(test_node_, name , tf_buffer, "base_link", "odom", tf2::durationFromSec(0.5), rclcpp::Duration::from_seconds(6.0));
  p->configure();
  rclcpp::Time start_time = test_node_->now();
  auto timeout = std::chrono::seconds(2);
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rclcpp::Time curr_time =  test_node_->get_clock()->now();
  std::vector<double> colors;
  p->getData(curr_time,colors );
  rclcpp::spin(test_node_->get_node_base_interface());

}



int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}