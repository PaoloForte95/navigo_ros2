cmake_minimum_required(VERSION 3.5)
project(gazebo_plugins_oru)

add_definitions(-std=c++14)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(orunav2_generic REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
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
ament_export_include_directories(include)
ament_export_dependencies(roscpp rospy orunav2_generic Boost)

###########
## Build ##
###########

include_directories(
  include
  rclcpp
  rclpy
  gazebo_plugins
  gazebo_msgs
  gazebo
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )
  
  
add_library(gazebo_ros_steer_drive src/gazebo_ros_steer_drive.cpp)
ament_target_dependencies(gazebo_ros_steer_drive rclcpp rclpy orunav2_generic gazebo gazebo_msgs gazebo_plugins)
target_link_libraries(gazebo_ros_steer_drive ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} )

add_library(gazebo_ros_tram_drive src/gazebo_ros_tram_drive.cpp)
ament_target_dependencies(gazebo_ros_tram_drive  rclcpp rclpy orunav2_generic gazebo gazebo_msgs gazebo_plugins)
target_link_libraries(gazebo_ros_tram_drive ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_ackermann_steer_drive src/gazebo_ros_ackermann_steer_drive.cpp)
ament_target_dependencies(gazebo_ros_ackermann_steer_drive rclcpp rclpy orunav2_generic gazebo gazebo_msgs gazebo_plugins)
target_link_libraries(gazebo_ros_ackermann_steer_drive ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_ackermann_dual_steer_drive src/gazebo_ros_ackermann_dual_steer_drive.cpp)
ament_target_dependencies(gazebo_ros_ackermann_dual_steer_drive rclcpp rclpy orunav2_generic gazebo gazebo_msgs gazebo_plugins)
target_link_libraries(gazebo_ros_ackermann_dual_steer_drive ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_fork_lifter src/gazebo_ros_fork_lifter.cpp)
ament_target_dependencies(gazebo_ros_fork_lifter rclcpp rclpy orunav2_generic gazebo gazebo_msgs gazebo_plugins )
target_link_libraries(gazebo_ros_fork_lifter ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_tippingbody_lifter src/gazebo_ros_tippingbody_lifter.cpp)
ament_target_dependencies(gazebo_ros_tippingbody_lifter rclcpp rclpy orunav2_generic gazebo gazebo_msgs gazebo_plugins)
target_link_libraries(gazebo_ros_tippingbody_lifter ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} )

add_library(gazebo_ros_jacks src/gazebo_ros_jacks.cpp)
ament_target_dependencies(gazebo_ros_jacks rclcpp rclpy orunav2_generic gazebo gazebo_msgs gazebo_plugins)
target_link_libraries(gazebo_ros_jacks ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

install(TARGETS 
  gazebo_ros_steer_drive
  gazebo_ros_tram_drive
  gazebo_ros_ackermann_steer_drive
  gazebo_ros_ackermann_dual_steer_drive
  gazebo_ros_fork_lifter
  gazebo_ros_tippingbody_lifter
  gazebo_ros_jacks
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  

ament_package()
