cmake_minimum_required(VERSION 3.5)
project(orunav2_msgs)

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
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME} 
   
  #Actions
   "action/GlobalPlannerSelector.action"
  #Messages

  #Services
   "srv/IsPlannerValid.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs action_msgs
)

install(
  FILES my_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})


ament_export_dependencies(rosidl_default_runtime)

ament_package()
