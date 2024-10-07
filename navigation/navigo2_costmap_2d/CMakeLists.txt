cmake_minimum_required(VERSION 3.8)
project(navigo2_costmap_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies
  nav2_costmap_2d
  pluginlib
)

set(library_name navigo2_costmap_2d)


add_library(${library_name} SHARED
  plugins/costmap_filters/area_filter.cpp

)

ament_target_dependencies(${library_name}
  ${dependencies}
)


install(TARGETS
${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  params
  maps
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_package()
