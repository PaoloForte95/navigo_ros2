cmake_minimum_required(VERSION 3.8)
project(navthon_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(ecceleron_conversions REQUIRED)
find_package(ecceleron REQUIRED COMPONENTS ecceleron::eclbas_external
                                           ecceleron::eclbas_common
                                           ecceleron::eclnav_mpc
)


include_directories(
  include
)

set(executable_name controller_server)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/controller_server.cpp
)


set(dependencies
  angles
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  nav2_msgs
  nav_2d_utils
  nav_2d_msgs
  nav2_util
  pluginlib
  nav2_core
  ecceleron_conversions
)


add_library(simple_goal_checker SHARED plugins/simple_goal_checker.cpp)
ament_target_dependencies(simple_goal_checker ${dependencies})

target_link_libraries(simple_goal_checker eclnav_mpc)

ament_target_dependencies(${library_name}
  ${dependencies}
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


ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "navthon_controller::ControllerServer")

install(TARGETS simple_goal_checker ${library_name}
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

ament_export_include_directories(include)
ament_export_libraries(${library_name}
simple_goal_checker)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)



ament_package()
