cmake_minimum_required(VERSION 3.5)
project(orunav_fork_control)
add_definitions(-std=c++17)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(orunav_msgs REQUIRED)
find_package(orunav2_generic REQUIRED)
find_package(orunav_rviz REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp std_msgs orunav_msgs orunav2_generic orunav_rviz visualization_msgs)


###########
## Build ##
###########

include_directories(
  include
  rclcpp
  std_msgs
  orunav_msgs
  orunav2_generic
  orunav_rviz
  visualization_msgs
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)



add_executable(fork_control_sim_node src/fork_control_sim_main.cpp)
ament_target_dependencies(fork_control_sim_node rclcpp std_msgs orunav_msgs orunav2_generic orunav_rviz visualization_msgs)


add_executable(fork_control_client src/fork_control_client_main.cpp)
ament_target_dependencies(fork_control_client rclcpp std_msgs orunav_msgs orunav2_generic orunav_rviz visualization_msgs)
target_link_libraries(fork_control_client ${Boost_LIBRARIES})

#############
## Install ##
#############

install(TARGETS fork_control_sim_node fork_control_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
   launch
   DESTINATION share
)

ament_package()
