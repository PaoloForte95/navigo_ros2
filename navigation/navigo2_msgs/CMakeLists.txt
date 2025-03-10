cmake_minimum_required(VERSION 3.5)
project(navigo2_msgs)

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
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME} 
   
  #Actions
  "action/GlobalPlannerSelector.action"
  "action/SmoothPath.action"
  #Messages
    "msg/WeatherState.msg"
    "msg/HumanDetection.msg"
  #Services
     "srv/GetWeatherCondition.srv"
     "srv/IsPlannerValid.srv"
     "srv/GetCurrentPath.srv"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs geometry_msgs sensor_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
