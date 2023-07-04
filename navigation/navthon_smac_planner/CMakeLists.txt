cmake_minimum_required(VERSION 3.8)
project(navthon_smac_planner)

add_compile_options(-std=c++17)
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

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(navthon_core REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(pluginlib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)
find_package(navthon_msgs REQUIRED)
find_package(navthon_generic REQUIRED)
find_package(navthon_util REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options thread regex)
find_package(OpenCV REQUIRED)
find_package(ecceleron_conversions REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
else()
  add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)
endif()

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(library_name navthon_smac_planner)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_smac_planner
  navthon_core
  nav2_core
  pluginlib
  angles
  eigen3_cmake_module
  navthon_msgs
  navthon_generic
  navthon_util
  ecceleron_conversions
  Boost
)


# A* plugin
add_library(${library_name}_astar SHARED
  src/planners/smac_planner_a_star.cpp
  src/smoother.cpp
)

target_link_libraries(${library_name}_astar ${OMPL_LIBRARIES} ${OpenMP_LIBRARIES} 
OpenMP::OpenMP_CXX)
target_include_directories(${library_name}_astar PUBLIC ${Eigen3_INCLUDE_DIRS})

ament_target_dependencies(${library_name}_astar
  ${dependencies}
)

# Lattice plugin
add_library(${library_name}_lattice SHARED
src/a_star_path_planner.cpp
src/a_star.cpp
src/ara_star_path_planner.cpp
src/collision_detector.cpp
src/disc_world_visualizer.cpp
src/map_generators.cpp
src/motion_primitive_data.cpp
src/motion_primitive_selector.cpp
src/path_finder.cpp
src/path_node.cpp
src/utils.cpp
src/vehicle_mission.cpp
src/world_occupancy_map.cpp
src/world_parameters.cpp
src/world.cpp
src/configuration/car_configuration.cpp
src/configuration/configuration.cpp
src/configuration/unicycle_configuration.cpp
src/configuration/lhd_configuration.cpp
src/model/car_model.cpp
src/model/lhd_model.cpp
src/model/unicycle_model.cpp
src/model/vehicle_model.cpp
src/planners/smac_planner_lattice.cpp
src/smoother.cpp
)

target_link_libraries(${library_name}_lattice ${OMPL_LIBRARIES} 
Boost::serialization 
Boost::thread 
Boost::program_options 
Boost::regex  
${OpenCV_LIBS})
target_include_directories(${library_name}_lattice PUBLIC ${Eigen3_INCLUDE_DIRS})

ament_target_dependencies(${library_name}_lattice
  ${dependencies}
)


pluginlib_export_plugin_description_file(nav2_core smac_plugin_astar.xml)
pluginlib_export_plugin_description_file(nav2_core smac_plugin_lattice.xml)

install(TARGETS ${library_name}_astar ${library_name}_lattice
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY lattice_primitives DESTINATION share/${PROJECT_NAME})
install(DIRECTORY lookup_tables DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()


ament_export_include_directories(include)
ament_export_libraries(${library_name}_astar ${library_name}_lattice)
ament_export_dependencies(${dependencies})
ament_package()
