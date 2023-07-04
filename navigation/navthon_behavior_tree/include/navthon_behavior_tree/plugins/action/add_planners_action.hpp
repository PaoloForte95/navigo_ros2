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

#ifndef NAVTHON_BEHAVIOR_TREE__PLUGINS__ACTION__ADD_PLANNERS_ACTION_HPP_
#define NAVTHON_BEHAVIOR_TREE__PLUGINS__ACTION__ADD_PLANNERS_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace navthon{
namespace behavior_tree{

class AddPlanners : public BT::ActionNodeBase
{
public:
  typedef std::vector<std::string> Planners;

  AddPlanners(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("input_planners", "A list of planners to add"),
      BT::OutputPort<Planners>("output_planners", "Set of planners after added any passed"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
  std::string string_planners_to_add;
  Planners planners_to_add, actual_planners_;

};

}
}  

#endif  // NAVTHON_BEHAVIOR_TREE__PLUGINS__ACTION__ADD_PLANNERS_ACTION_HPP_
