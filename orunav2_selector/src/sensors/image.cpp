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

#include <cmath>
#include <chrono>
#include <functional>

#include "orunav2_selector/sensors/image.hpp"
#include "nav2_util/node_utils.hpp"


namespace orunav2_selector
{

Image::Image(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Image", source_name_.c_str());
}

Image::~Image()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Image", source_name_.c_str());
  data_sub_.reset();
}

void Image::configure()
{
  auto node = node_.lock();
  logger_ = node->get_logger();
  
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;
  
  // Get common parameters
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".topic",rclcpp::ParameterValue("/intel_realsense_r200_depth/image_raw"));  // Set deafult topic for camera
  source_topic = node->get_parameter(source_name_ + ".topic").as_string();
  
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".max_distance_range", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".min_distance_range", rclcpp::ParameterValue(0.1));

  max_distance_range_ = node->get_parameter(source_name_ + ".max_distance_range").as_double();
  min_distance_range_ = node->get_parameter(source_name_ + ".min_distance_range").as_double();

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default

  //callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  //callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  //callback_group_executor_->add_callback_group(callback_group_, node->get_node_base_interface());
  //rclcpp::SubscriptionOptions options;
  //options.callback_group = callback_group_;
  client_node_ = rclcpp::Node::make_shared("client_node");
  data_sub_ = node->create_subscription<sensor_msgs::msg::Image>(source_topic, scan_qos,std::bind(&Image::dataCallback, this, std::placeholders::_1));
  weather_condition_client_ = client_node_->create_client<orunav2_msgs::srv::GetWeatherCondition>("get_weather_condition",  rclcpp::ServicesQoS().get_rmw_qos_profile());
  RCLCPP_INFO(logger_, "[%s]: Camera subscribing to topic: %s", source_name_.c_str(), source_topic.c_str() );
}


void Image::getData(const rclcpp::Time & curr_time, std::vector<geometry_msgs::msg::Point> & data) const
{
    //TODO
}

void Image::getData(const rclcpp::Time & curr_time, std::vector<double> & data) const
{ 

    //auto client_node = rclcpp::Node::make_shared("client_node");

    //Get the weather condition

    auto request = std::make_shared<orunav2_msgs::srv::GetWeatherCondition::Request>();
    request->image = data_;
    auto result = weather_condition_client_->async_send_request(request);
    //auto future = callback_group_executor_->spin_until_future_complete(result, std::chrono::seconds(1));
    auto future = rclcpp::spin_until_future_complete(client_node_, result, std::chrono::seconds(5));
    if (future != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(logger_, "Failed to call service %s", weather_condition_client_->get_service_name());
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    RCLCPP_INFO(logger_, "Got weather condition!It is %ld ", result.get()->condition);

}

void Image::dataCallback(sensor_msgs::msg::Image msg)
{
  data_ = msg;
}

}  // namespace orunav2_selector
