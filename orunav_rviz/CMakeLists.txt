cmake_minimum_required(VERSION 3.8)
project(orunav_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(orunav2_generic REQUIRED)
find_package(orunav2_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
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
ament_export_dependencies(Boost)
ament_export_dependencies(orunav2_generic orunav2_conversions tf2 visualization_msgs)
ament_export_include_directories(include)
include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  orunav2_generic
  orunav2_conversions
  tf2
  visualization_msgs
  )
#############
## Install ##
#############
# Install config and launch files.
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
ament_package()
