cmake_minimum_required(VERSION 3.8)
project(orunav_geometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(orunav2_generic REQUIRED)
find_package(angles REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
ament_export_dependencies(Boost angles orunav2_generic)
ament_export_libraries(${PROJECT_NAME})

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  orunav2_generic
  angles
  )

## Declare a cpp library
add_library(orunav_geometry SHARED
  src/polygon.cpp 
  src/robot_model_2d.cpp 
  src/pallet_model_2d.cpp
  src/b_spline.cpp
  src/b_spline2d.cpp
  )

ament_target_dependencies(orunav_geometry
  angles
  orunav2_generic
  )

  
## Declare a cpp executable
add_executable(polygon_test test/polygon_test.cpp)
ament_target_dependencies(polygon_test
  orunav2_generic
  angles
  )



  
add_executable(polygon2_test test/polygon2_test.cpp)
ament_target_dependencies(polygon2_test
  orunav2_generic
  angles
  )
target_link_libraries(polygon2_test -lboost_serialization orunav_geometry)



#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION include/${PROJECT_NAME}
       )

ament_package()
