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


#ifndef NAVIGO2_WEATHER_DETECTOR__WEATHER_DETECTOR_NODE_HPP_
#define NAVIGO2_WEATHER_DETECTOR__WEATHER_DETECTOR_NODE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"


#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "navigo2_selector/sensors/image.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "navigo2_msgs/srv/get_weather_condition.hpp"
#include "navigo2_msgs/msg/weather_state.hpp"
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace navigo2_weather_detection
{
    class WeatherDetector : public rclcpp_lifecycle::LifecycleNode
{
    public:
  /**
   * @brief Constructor for the navigo2_weather_detection::WeatherDetector
   * @param options Additional options to control creation of the node.
   */
  explicit WeatherDetector(const std::string & node_name,
        const std::string & ns = "",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the navigo2_weather_detection::WeatherDetector
   */
  ~WeatherDetector();
  /**
   * @brief: Initializes and obtains ROS-parameters, creates main subscribers and publishers
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Activates LifecyclePublishers and main processor, creates bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Deactivates LifecyclePublishers and main processor, destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Resets all subscribers/publishers
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called in shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Main processing routine
   */
  void process();

  /**
     * @brief Supporting routine creating and configuring data source
     * @param base_frame_id Robot base frame ID
     * @param odom_frame_id Odometry frame ID. Used as global frame to get
     * source->base time inerpolated transform.
     * @param transform_tolerance Transform tolerance
     * @param source_timeout Maximum time interval in which data is considered valid
     * @return True if source was configured successfully or false in failure case
     */
    bool configureSources(
      const std::string & base_frame_id,
      const std::string & odom_frame_id,
      const tf2::Duration & transform_tolerance,
      const rclcpp::Duration & source_timeout);


 /**
   * @brief Image data callback
   * @param msg Shared pointer to Image message
   */
  void dataCallback(sensor_msgs::msg::Image msg);



// ----- Variables -----

/// @brief Laser scanner data subscriber
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr data_sub_;
/// @brief Weather condition client 
rclcpp::Client<navigo2_msgs::srv::GetWeatherCondition>::SharedPtr weather_condition_client_;
rclcpp::Node::SharedPtr client_node_;
rclcpp_lifecycle::LifecyclePublisher<navigo2_msgs::msg::WeatherState>::SharedPtr weather_condition_pub_;
rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;

std::vector<std::shared_ptr<navigo2_selector::Image>> source_;


/// @brief Latest data obtained from camera
sensor_msgs::msg::Image data_;
nav2_msgs::msg::SpeedLimit limited_speed;

int rate_;
int last_evaluation_time_ = 0;
double slowdown_ratio_;
bool percentage_;

rclcpp::Logger logger_{rclcpp::get_logger("WeatherDetector")};

};// class WeatherDetector

}// namespace navigo2_weather_detection

#endif  // NAVIGO2_WEATHER_DETECTOR__WEATHER_DETECTOR_NODE_HPP_
