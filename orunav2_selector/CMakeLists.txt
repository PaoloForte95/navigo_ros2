cmake_minimum_required(VERSION 3.8)
project(orunav2_selector)
add_compile_options(-std=c++17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(orunav2_core REQUIRED)
find_package(orunav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_util REQUIRED)



include_directories(
  include
)

set(executable_name global_planner_selector_server)
set(library_name orunav2_selector)
set(library_name_server ${executable_name}_core)


set(dependencies_server
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  visualization_msgs
  nav2_util
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  pluginlib
  orunav2_core
  orunav2_msgs
)

#SELECTORS
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  orunav2_core
  orunav2_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  pluginlib
  nav2_smac_planner
)

#Selector server
############
add_library(${library_name_server} SHARED
  src/global_planner_selector_server.cpp
)

ament_target_dependencies(${library_name_server}
  ${dependencies_server}
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name_server})

ament_target_dependencies(${executable_name}
  ${dependencies_server}
)

rclcpp_components_register_nodes(${library_name_server} "orunav2_selector::PlannerSelectorServer")
##############################


# Costmap Selector
add_library(${library_name}_costmap SHARED
  src/costmap_selector.cpp

)

ament_target_dependencies(${library_name}_costmap
  ${dependencies}
)



pluginlib_export_plugin_description_file(orunav2_core costmap_selector.xml)


##############
# Install 
#############
#Server
install(TARGETS ${library_name_server}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
#Selectors
install(TARGETS ${library_name}_costmap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
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



ament_export_include_directories(include)
#Server
ament_export_libraries(${library_name_server})
ament_export_dependencies(${dependencies_server})
#Selectors
ament_export_libraries(${library_name}_costmap)
ament_export_dependencies(${dependencies})

ament_package()
