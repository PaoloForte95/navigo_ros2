cmake_minimum_required(VERSION 3.8)
project(orunav2_behavior_tree)

add_compile_options(-std=c++17)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(orunav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)


include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  nav2_msgs
  orunav2_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
  nav2_costmap_2d
)

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(orunav2_remove_unfeasible_planner_action_bt_node SHARED plugins/action/remove_unfeasible_planner_action.cpp)
list(APPEND plugin_libs orunav2_remove_unfeasible_planner_action_bt_node)

add_library(orunav2_add_planners_action_bt_node SHARED plugins/action/add_planners_action.cpp)
list(APPEND plugin_libs orunav2_add_planners_action_bt_node)

add_library(orunav2_global_planner_selector_action_bt_node SHARED plugins/action/global_planner_selector_node.cpp)
list(APPEND plugin_libs orunav2_global_planner_selector_action_bt_node)

add_library(orunav2_is_planner_updated_condition_bt_node SHARED plugins/condition/is_planner_updated_condition.cpp)
list(APPEND plugin_libs orunav2_is_planner_updated_condition_bt_node)

add_library(orunav2_is_planner_equal_condition_bt_node SHARED plugins/condition/is_planner_equal_condition.cpp)
list(APPEND plugin_libs orunav2_is_planner_equal_condition_bt_node)

add_library(orunav2_is_path_computed_condition_bt_node SHARED plugins/condition/is_path_computed_condition.cpp)
list(APPEND plugin_libs orunav2_is_path_computed_condition_bt_node)

add_library(orunav2_distance_waypoints_controller_bt_node SHARED plugins/decorator/distance_to_waypoints_controller.cpp)
list(APPEND plugin_libs orunav2_distance_waypoints_controller_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name} 
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_trees/orunav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
