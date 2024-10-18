cmake_minimum_required(VERSION 3.8)
project(navigo2_smoother)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(navigo2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigo2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

set(executable_name smoother_server)
set(library_name ${executable_name}_core)

set(dependencies
  angles
  rclcpp
  rclcpp_components
  rclcpp_action
  rclcpp_components
  std_msgs
  nav2_msgs
  navigo2_msgs
  nav_msgs
  nav_2d_utils
  nav_2d_msgs
  nav2_util
  navigo2_core
  pluginlib
)

# Main library
add_library(${library_name} SHARED
  src/smoother_server.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

# Main executable
add_executable(${executable_name}
  src/main.cpp
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name} ${library_name})

# Simple Smoother plugin
add_library(simple_smoother SHARED
  src/simple_smoother.cpp
)
ament_target_dependencies(simple_smoother
  ${dependencies}
)

pluginlib_export_plugin_description_file(navigo2_core smoother_plugins.xml)


rclcpp_components_register_nodes(${library_name} "navigo2_smoother::SmootherServer")


install(
  TARGETS ${library_name} simple_smoother
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
ament_export_libraries(${library_name} simple_smoother)
ament_export_dependencies(${dependencies})

ament_package()
