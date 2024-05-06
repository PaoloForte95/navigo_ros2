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


#ifndef NAVTHON_HUMAN_DETECTION__HUMAN_DETECTOR_NODE_HPP_
#define NAVTHON_HUMAN_DETECTION__HUMAN_DETECTOR_NODE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"


#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"

#include "navthon_selector/sensors/image.hpp"

#include "navthon_msgs/msg/human_detection.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

namespace navthon_human_detection
{
    class HumanDetector : public nav2_util::LifecycleNode
{
    public:
  /**
   * @brief Constructor for the navthon_human_detection::HumanDetector
   * @param options Additional options to control creation of the node.
   */
  explicit HumanDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the navthon_human_detection::HumanDetector
   */
  ~HumanDetector();

  protected:
  /**
   * @brief: Initializes and obtains ROS-parameters, creates main subscribers and publishers
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Activates LifecyclePublishers and main processor, creates bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Deactivates LifecyclePublishers and main processor, destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Resets all subscribers/publishers
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called in shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

 /**
   * @brief Image data callback
   * @param msg Shared pointer to Image message
   */
  void dataCallback(navthon_msgs::msg::HumanDetection msg);



// ----- Variables -----

/// @brief Human detector data subscriber
rclcpp::Subscription<navthon_msgs::msg::HumanDetection>::SharedPtr data_sub_;
rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
/// @brief Whether main routine is active
bool process_active_;

navthon_msgs::msg::HumanDetection human_detection_;
nav2_msgs::msg::SpeedLimit limited_speed;

int rate_;
double slowdown_ratio_;
bool percentage_;

rclcpp::Logger logger_{rclcpp::get_logger("HumanDetector")};

};// class HumanDetector

}

#endif 
