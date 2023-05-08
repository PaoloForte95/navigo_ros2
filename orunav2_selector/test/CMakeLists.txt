#C++ Tests
ament_add_gtest(sources_image_test sources_image_test.cpp)
ament_target_dependencies(sources_image_test
  ${dependencies}
)
target_link_libraries(sources_image_test
  sources
  ${library_name}
)

#Python Tests
set(_pytest_tests
  weather_detection_test.py
)
foreach(_test_path ${_pytest_tests})
  get_filename_component(_test_name ${_test_path} NAME_WE)
  ament_add_pytest_test(${_test_name} ${_test_path}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endforeach()