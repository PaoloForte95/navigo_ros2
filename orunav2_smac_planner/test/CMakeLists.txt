# Test SMAC lattice
ament_add_gtest(test_smac_planner_lattice
test_smac_planner_lattice.cpp
)
ament_target_dependencies(test_smac_planner_lattice
  ${dependencies}
)
target_link_libraries(test_smac_planner_lattice
  ${library_name}_lattice
)

# Test SMAC A*
ament_add_gtest(test_smac_planner_astar
test_smac_planner_astar.cpp
)
ament_target_dependencies(test_smac_planner_astar
  ${dependencies}
)
target_link_libraries(test_smac_planner_astar
  ${library_name}_astar
)


