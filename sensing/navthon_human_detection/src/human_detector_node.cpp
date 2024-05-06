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


#include "navthon_human_detection/human_detector_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace navthon_human_detection
{
HumanDetector::HumanDetector(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("human_detector", "", options),
  process_active_(false)
{
}

HumanDetector::~HumanDetector()
{
}


nav2_util::CallbackReturn
HumanDetector::on_configure(const rclcpp_lifecycle::State & /*state*/)

{
    auto node = shared_from_this();

    logger_ = node->get_logger();
    RCLCPP_INFO(logger_, "Configuring human detector");
    rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default

    std::string topic, base_frame_id, camera_frame_id,speed_limit_topic;
    // Obtaining ROS parameters
    nav2_util::declare_parameter_if_not_declared(node, "rate", rclcpp::ParameterValue(1));
    nav2_util::declare_parameter_if_not_declared(node, "topic", rclcpp::ParameterValue("human_detection"));
    nav2_util::declare_parameter_if_not_declared(node, "base_frame_id", rclcpp::ParameterValue("base_link"));
    nav2_util::declare_parameter_if_not_declared(node, "camera_frame", rclcpp::ParameterValue("camera_link"));
    nav2_util::declare_parameter_if_not_declared(node, "speed_limit_topic", rclcpp::ParameterValue("/speed_limit"));
    nav2_util::declare_parameter_if_not_declared(node, "slowdown_ratio", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, "percentage", rclcpp::ParameterValue(true));
  
  
    topic = node->get_parameter("topic").as_string();
    base_frame_id = node->get_parameter("base_frame_id").as_string();
    camera_frame_id =  node->get_parameter("camera_frame").as_string();
    speed_limit_topic = node->get_parameter("speed_limit_topic").as_string();
    slowdown_ratio_ = node->get_parameter("slowdown_ratio").as_double();
    percentage_ =node->get_parameter("percentage").as_bool();
    

    rate_ = node->get_parameter("rate").as_int();

    data_sub_ = node->create_subscription<navthon_msgs::msg::HumanDetection>(topic, scan_qos,std::bind(&HumanDetector::dataCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "Subscribing to topic: %s", topic.c_str() );
    speed_limit_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>(speed_limit_topic, 1);


  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn
HumanDetector::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating weather detector");

   // Activating lifecycle publisher
  speed_limit_pub_->on_activate();


  // Activating main worker
  process_active_ = true;

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
HumanDetector::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating weather detector");


  // Deactivating lifecycle publishers
  speed_limit_pub_->on_deactivate();

  // Deactivating main worker
  process_active_ = false;

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
HumanDetector::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up weather detector");
  
  //Pubs
  speed_limit_pub_.reset();

  //Subs
  data_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
HumanDetector::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}


void HumanDetector::dataCallback(navthon_msgs::msg::HumanDetection msg)
{
  human_detection_ = msg;
  std::unique_ptr<nav2_msgs::msg::SpeedLimit> speed_limit_msg =std::make_unique<nav2_msgs::msg::SpeedLimit>();
  bool limit_velocity = false;
  if(msg.human_detected){
    limit_velocity = true;
  }
  if(limit_velocity){
    int number_humans = msg.human_number;
    if(percentage_){
      RCLCPP_INFO(logger_, "Detected %d humans....Reducing speed of %.2f%s ",number_humans,  slowdown_ratio_, "%");
      
    }
    else{
      RCLCPP_INFO(logger_, "Detected %d humans....Reducing speed to %.2f ",number_humans, slowdown_ratio_);
    }
    speed_limit_msg->percentage = percentage_;
    speed_limit_msg->speed_limit = slowdown_ratio_;
    speed_limit_pub_->publish(std::move(speed_limit_msg));
  }
  else {
    RCLCPP_INFO(logger_, "No human detected ... No need to reduce velocity!");
  }
   
 

}


}// namespace navthon_weather_detector
