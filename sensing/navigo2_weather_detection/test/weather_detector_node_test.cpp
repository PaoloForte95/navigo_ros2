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
#include "navigo2_selector/sensors/image.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/lifecycle_node.hpp"
#include "navigo2_weather_detection/weather_detector_node.hpp"


class WeatherDetectorTestNode : public navigo2_weather_detection::WeatherDetector
{
public:
  void start()
  {
      RCLCPP_INFO(get_logger(), "Starting weather test");
      ASSERT_EQ(on_configure(get_current_state()), CallbackReturn::SUCCESS);
      ASSERT_EQ(on_activate(get_current_state()), CallbackReturn::SUCCESS);
       
  }

  void stop()
  {
    ASSERT_EQ(on_deactivate(get_current_state()), CallbackReturn::SUCCESS);
    ASSERT_EQ(on_cleanup(get_current_state()), CallbackReturn::SUCCESS);
    ASSERT_EQ(on_shutdown(get_current_state()), CallbackReturn::SUCCESS);
  }

};


class TestNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  TestNode()
  : rclcpp_lifecycle::LifecycleNode("test_node")
  {
     wd_ = std::make_shared<WeatherDetectorTestNode>("weather_detector_node");
  }

  ~TestNode()
  {
  }
  

    // CollisionMonitor node
  std::shared_ptr<WeatherDetectorTestNode> wd_;

};


TEST(TestNode, weather_detector_node_test)
{  
    
  auto test_node_ = std::make_shared<TestNode>();
  auto wd_node = test_node_->wd_;
  wd_node->start();
  rclcpp::Time start_time = wd_node->now();
  auto timeout = std::chrono::seconds(2);
  while (rclcpp::ok() && wd_node->now() - start_time <= rclcpp::Duration(timeout)) {
    rclcpp::spin_some(wd_node->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rclcpp::Time curr_time =  wd_node->get_clock()->now();
  rclcpp::spin(wd_node->get_node_base_interface());

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