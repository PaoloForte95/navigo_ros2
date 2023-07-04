// Copyright (c) 2022 Paolo Forte
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

#ifndef NAVTHON_UTIL__CONVERSION_UTILS_HPP_
#define NAVTHON_UTIL__CONVERSION_UTILS_HPP_

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <navthon_generic/types.hpp>

namespace navthon_util
{
namespace conversion_utils
{

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
  {
  	tf2::Quaternion q;
  	q.setRPY(0, 0, yaw);
  	return tf2::toMsg(q);
  }	

  nav_msgs::msg::Path createPathMsgFromPathInterface(const navthon_generic::PathInterface &path)
  {
    nav_msgs::msg::Path p;
    for (size_t i = 0; i < path.sizePath(); i++)
      {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.orientation = createQuaternionMsgFromYaw(path.getPose2d(i)(2));
        ps.pose.position.x = path.getPose2d(i)(0);
        ps.pose.position.y = path.getPose2d(i)(1);
        ps.pose.position.z = 0.;
        p.poses.push_back(ps);
      }
    return p;
  }

}  
}  

#endif  // 