// Copyright (c) 2019 Intel Corporation
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

#ifndef NAVIGO2_RVIZ_PLUGINS__NAVNAVIGO2_PANEL_HPP_
#define NAVIGO2_RVIZ_PLUGINS__NAVNAVIGO2_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>



#include "nav2_rviz_plugins/nav2_panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "navigo2_msgs/msg/weather_state.hpp"

class QPushButton;
using namespace nav2_rviz_plugins;

namespace navigo2_rviz_plugins
{

/// Panel to interface to the navigo2 stack
class NavthonPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NavthonPanel(QWidget * parent = 0);
  virtual ~NavthonPanel();
  void onInitialize() override;

private:
  QLabel * selected_planner_indicator_{nullptr};
  QLabel * selected_controller_indicator_{nullptr};
  QLabel * weather_detection_indicator_{nullptr};
  // Navigation subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_planner_sub_;
  rclcpp::Subscription<navigo2_msgs::msg::WeatherState>::SharedPtr detected_weather_sub_;


  // create label string from string msg
  void getSelectedPlannerLabel(const std_msgs::msg::String::SharedPtr msg);

  void getWeatherDetected(const navigo2_msgs::msg::WeatherState::SharedPtr msg);

};


} 

#endif  
