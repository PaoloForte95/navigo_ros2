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


#include "navthon_weather_detector/weather_detector_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace navthon_weather_detector
{
WeatherDetector::WeatherDetector(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("weather_detector", "", options),
  process_active_(false)
{
}

WeatherDetector::~WeatherDetector()
{
  source_.clear();
}


nav2_util::CallbackReturn
WeatherDetector::on_configure(const rclcpp_lifecycle::State & /*state*/)

{
    auto node = shared_from_this();

    logger_ = node->get_logger();
    RCLCPP_INFO(logger_, "Configuring weather detector");
    rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default

    std::string image_topic, service_name, base_frame_id, odom_frame_id,speed_limit_topic;
    std::string source_type = "image";
    // Obtaining ROS parameters
    nav2_util::declare_parameter_if_not_declared(node, "rate", rclcpp::ParameterValue(1));
    nav2_util::declare_parameter_if_not_declared(node, "image_topic", rclcpp::ParameterValue("/intel_realsense_r200_depth/image_raw"));
    nav2_util::declare_parameter_if_not_declared(node, "service_name", rclcpp::ParameterValue("/get_weather_condition"));
    nav2_util::declare_parameter_if_not_declared(node, "base_frame_id", rclcpp::ParameterValue("base_link"));
    nav2_util::declare_parameter_if_not_declared(node, "odom_frame_id", rclcpp::ParameterValue("world"));
    nav2_util::declare_parameter_if_not_declared(node, "speed_limit_topic", rclcpp::ParameterValue("/speed_limit"));
    nav2_util::declare_parameter_if_not_declared(node, "slowdown_ratio", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, "percentage", rclcpp::ParameterValue(true));
  
  
    image_topic = node->get_parameter("image_topic").as_string();
    service_name = node->get_parameter("service_name").as_string();
    base_frame_id = node->get_parameter("base_frame_id").as_string();
    odom_frame_id =  node->get_parameter("odom_frame_id").as_string();
    speed_limit_topic = node->get_parameter("speed_limit_topic").as_string();
    slowdown_ratio_ = node->get_parameter("slowdown_ratio").as_double();
    percentage_ =node->get_parameter("percentage").as_bool();
    

    rate_ = node->get_parameter("rate").as_int();

    data_sub_ = node->create_subscription<sensor_msgs::msg::Image>(image_topic, scan_qos,std::bind(&WeatherDetector::dataCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "[%s]: Camera subscribing to topic: %s", source_type.c_str(), image_topic.c_str() );
    weather_condition_pub_ = this->create_publisher<navthon_msgs::msg::WeatherState>("/weather_condition", 1);
    speed_limit_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>(speed_limit_topic, 1);
    tf2::Duration transform_tolerance = tf2::durationFromSec(0.5);
    rclcpp::Duration source_timeout = rclcpp::Duration::from_seconds(6.0);
    std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
    tf_buffer->setCreateTimerInterface(timer_interface);

    
    //Configure Image source 
    RCLCPP_INFO(logger_,"Adding source of type %s.", source_type.c_str());
    std::shared_ptr<navthon_selector::Image> p = std::make_shared<navthon_selector::Image>(node, source_type, tf_buffer, base_frame_id, odom_frame_id, transform_tolerance, source_timeout);
    p->configure();
    source_.push_back(p);

    client_node_ = rclcpp::Node::make_shared("client_node");
    weather_condition_client_ = client_node_->create_client<navthon_msgs::srv::GetWeatherCondition>(service_name,  rclcpp::ServicesQoS().get_rmw_qos_profile());
     RCLCPP_INFO(logger_, "[%s]: Creating client for service: %s", source_type.c_str(), service_name.c_str() );



  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn
WeatherDetector::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating weather detector");

   // Activating lifecycle publisher
  weather_condition_pub_->on_activate();
  speed_limit_pub_->on_activate();


  // Activating main worker
  process_active_ = true;

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WeatherDetector::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating weather detector");


  // Deactivating lifecycle publishers
  weather_condition_pub_->on_deactivate();
  speed_limit_pub_->on_deactivate();

  // Deactivating main worker
  process_active_ = false;

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WeatherDetector::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up weather detector");
  
  //Pubs
  weather_condition_pub_.reset();
  speed_limit_pub_.reset();

  //Subs
  data_sub_.reset();
  weather_condition_client_.reset();

  source_.clear();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WeatherDetector::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}


void WeatherDetector::process()
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Do nothing if main worker in non-active state
  if (!process_active_) {
    return;
  }
  // Creating the request for the service
  auto request = std::make_shared<navthon_msgs::srv::GetWeatherCondition::Request>();
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
  auto state = result.get()->condition;
  auto state_str = "";
  bool limit_velocity = false;
  switch (state.condition) {
    case 0:
      state_str = "CLEAR";
      break;
    case 1:
      state_str = "FOG";
      break;
    case 2:
      state_str = "RAIN";
       limit_velocity= true;
      break;
    case 3:
      state_str = "SNOW";
      limit_velocity= true;
      break;
  }
  RCLCPP_INFO(logger_, "Got weather condition!It is %s ", state_str);

  std::unique_ptr<navthon_msgs::msg::WeatherState> weather_state_msg =std::make_unique<navthon_msgs::msg::WeatherState>();
  weather_state_msg->condition = state.condition;
  weather_condition_pub_->publish(std::move(weather_state_msg));
  std::unique_ptr<nav2_msgs::msg::SpeedLimit> speed_limit_msg =std::make_unique<nav2_msgs::msg::SpeedLimit>();
  if(limit_velocity){
    if(percentage_){
      RCLCPP_INFO(logger_, "Detected %s....Reducing speed of %.2f%s ",state_str,  slowdown_ratio_, "%");
      
    }
    else{
      RCLCPP_INFO(logger_, "Detected %s....Reducing speed to %.2f ",state_str, slowdown_ratio_);
    }
    speed_limit_msg->percentage = percentage_;
    speed_limit_msg->speed_limit = slowdown_ratio_;
    speed_limit_pub_->publish(std::move(speed_limit_msg));
  }
  else {
    RCLCPP_INFO(logger_, "Detected %s.... No need to reduce velocity!",state_str);
  }
   


}


void WeatherDetector::dataCallback(sensor_msgs::msg::Image msg)
{
  data_ = msg;
  auto time_diff = msg.header.stamp.sec - last_evaluation_time_;
  if(time_diff > rate_){
    process();
    last_evaluation_time_ =  msg.header.stamp.sec;
  }
 

}


}// namespace navthon_weather_detector
