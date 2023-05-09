cmake_minimum_required(VERSION 3.8)
project(orunav2_weather_detector)

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
find_package(orunav2_selector REQUIRED)

include_directories(
  include
)

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
  orunav2_selector
  rclpy
)

set(executable_name weather_detector)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)
#Weather Detector server
############
add_library(${library_name} SHARED
  src/weather_detector_node.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})


rclcpp_components_register_nodes(${library_name} "orunav2_weather_detector::WeatherDetector")


##############
# Install 
#############

install(TARGETS ${library_name}
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
ament_export_dependencies(${dependencies})
ament_package()
