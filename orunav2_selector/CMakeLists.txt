cmake_minimum_required(VERSION 3.8)
project(orunav2_selector)
add_compile_options(-std=c++17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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
find_package(nav2_collision_monitor REQUIRED)
find_package(rclpy REQUIRED)



include_directories(
  include
)

set(executable_name selector_server)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)


set(dependencies
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
  nav2_smac_planner
  nav2_collision_monitor
)


#Selector server
############
add_library(${library_name} SHARED
  src/global_planner_selector_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)


##############################
#Sources 
add_library(sources SHARED
  src/sensors/scan.cpp
  src/sensors/source.cpp
  src/sensors/image.cpp
)
ament_target_dependencies(sources
  ${dependencies}
)

# Costmap Selector
add_library(costmap_selector SHARED
  src/selectors/costmap_selector.cpp
)

ament_target_dependencies(costmap_selector
  ${dependencies}
)
target_link_libraries(costmap_selector sources)

ament_target_dependencies(${executable_name}
  ${dependencies_server}
)
target_link_libraries(${executable_name} ${library_name})



rclcpp_components_register_nodes(${library_name} "orunav2_selector::PlannerSelectorServer")



##############
# Install 
#############
#Server
install(TARGETS ${library_name} costmap_selector sources
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  src/detector/weather_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  #Tests
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test)


endif()



ament_export_include_directories(include)
ament_export_libraries(costmap_selector
${library_name}
sources)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(orunav2_core costmap_selector.xml)

ament_package()
