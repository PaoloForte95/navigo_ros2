cmake_minimum_required(VERSION 3.8)
project(navigo2_rviz_plugins)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(navigo2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_rviz_plugins REQUIRED)



set(navigo2_rviz_plugins_headers
  include/navigo2_rviz_plugins/navigo2_panel.hpp
)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/navigo2_panel.cpp
  ${navigo2_rviz_plugins_headers}
)

set(dependencies
  geometry_msgs
  nav2_util
  nav2_lifecycle_manager
  nav2_msgs
  navigo2_msgs
  nav_msgs
  pluginlib
  Qt5
  rclcpp
  rclcpp_lifecycle
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  std_msgs
  tf2_geometry_msgs
  nav2_rviz_plugins
)


ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${library_name}
  rviz_common::rviz_common
)

target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)


install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  map_msgs
  nav_msgs
  rclcpp
  nav2_rviz_plugins
)

ament_package()
