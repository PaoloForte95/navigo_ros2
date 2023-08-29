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

#include "navthon_rviz_plugins/navthon_panel.hpp"

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include <string>

#include "nav2_rviz_plugins/goal_common.hpp"
#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace navthon_rviz_plugins
{


NavthonPanel::NavthonPanel(QWidget * parent)
: Panel(parent)
{
    selected_planner_indicator_ = new QLabel;
    selected_controller_indicator_ = new QLabel;

    const QString selected_planner_unknown("<table><tr><td width=100><b>Planner:</b></td><td>unknown</td></tr></table>");
    const QString selected_controller_unknown("<table><tr><td width=100><b>Controller:</b></td><td>unknown</td></tr></table>");

    selected_planner_indicator_->setText(selected_planner_unknown);
    selected_controller_indicator_->setText(selected_controller_unknown);

    selected_planner_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    selected_controller_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);


    // Lay out the items in the panel
    QVBoxLayout * main_layout = new QVBoxLayout;
    main_layout->addWidget(selected_planner_indicator_);
    main_layout->addWidget(selected_controller_indicator_);


    main_layout->setContentsMargins(10, 10, 10, 10);
    setLayout(main_layout);

}

NavthonPanel::~NavthonPanel()
{
}

void
NavthonPanel::onInitialize()
{  

    
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // create action feedback subscribers
  selected_planner_sub_ =
    node->create_subscription<std_msgs::msg::String>(
    "selected_planner",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavthonPanel::getSelectedPlannerLabel, this, std::placeholders::_1));

}

void NavthonPanel::getSelectedPlannerLabel(const std_msgs::msg::String::SharedPtr msg)
{
    if(msg->data.empty()){
        msg->data = "unknown";
    }
    QString planner;
    selected_planner_indicator_->setText( QString(std::string("<table><tr><td width=100><b>Planner:</b></td>"
    "<td><font color=green>"+ msg->data + "</color></td></tr></table>").c_str()));
    selected_controller_indicator_->setText( QString(std::string("<table><tr><td width=100><b>Controller:</b></td>"
    "<td><font color=green>Spline Controller</color></td></tr></table>").c_str()));
}


}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(navthon_rviz_plugins::NavthonPanel, rviz_common::Panel)
