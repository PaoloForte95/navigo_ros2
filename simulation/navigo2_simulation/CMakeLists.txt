cmake_minimum_required(VERSION 3.8)
project(navigo2_simulation)

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
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(navigo2_generic REQUIRED)
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

include_directories(
  #include
  rclcpp
  rclpy
  gazebo_plugins
  gazebo_msgs
  gazebo
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  
)


#add_library(gazebo_ros_tram_drive src/gazebo_ros_tram_drive.cpp)
#ament_target_dependencies(gazebo_ros_tram_drive  rclcpp rclpy navigo2_generic gazebo gazebo_msgs gazebo_plugins)
#target_link_libraries(gazebo_ros_tram_drive ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_library(gazebo_ros_fork_lifter src/gazebo_ros_fork_lifter.cpp)
#ament_target_dependencies(gazebo_ros_fork_lifter rclcpp rclpy navigo2_generic gazebo gazebo_msgs gazebo_plugins )
#target_link_libraries(gazebo_ros_fork_lifter ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_library(gazebo_ros_tippingbody_lifter src/gazebo_ros_tippingbody_lifter.cpp)
#ament_target_dependencies(gazebo_ros_tippingbody_lifter rclcpp rclpy navigo2_generic gazebo gazebo_msgs gazebo_plugins)
#target_link_libraries(gazebo_ros_tippingbody_lifter ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} )

#add_library(gazebo_ros_jacks src/gazebo_ros_jacks.cpp)
#ament_target_dependencies(gazebo_ros_jacks rclcpp rclpy navigo2_generic gazebo gazebo_msgs gazebo_plugins)
#target_link_libraries(gazebo_ros_jacks ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

#install(TARGETS 
  #gazebo_ros_tram_drive
  #gazebo_ros_fork_lifter
  #gazebo_ros_tippingbody_lifter
  #gazebo_ros_jacks
  #ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
  #RUNTIME DESTINATION lib/${PROJECT_NAME}
  #)



# Install config and launch files.
install(DIRECTORY
  launch
  config
  params
  models
  DESTINATION share/${PROJECT_NAME}/
)

#install(DIRECTORY include/
  #DESTINATION include/
#)


#ament_export_include_directories(
 # include
#)

ament_package()
