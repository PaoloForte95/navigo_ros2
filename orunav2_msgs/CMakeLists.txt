cmake_minimum_required(VERSION 3.5)
project(orunav2_msgs)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
   
  #Actions
   "action/GlobalPlannerSelector.action"
   "action/FollowPath.action"
   "action/ComputePathToPose.action"
   "action/ComputePathThroughPoses.action"
   "action/SmoothPath.action"
   "action/NavigateToPose.action"
  #Messages
    "msg/Path.msg"
    "msg/PathPoint.msg"
  #Services
   "srv/IsPlannerValid.srv"
   "srv/IsPathValid.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs action_msgs 
)

install(
  FILES my_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})


ament_export_dependencies(rosidl_default_runtime)

ament_package()
