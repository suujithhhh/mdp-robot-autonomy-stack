add_library(mdp_planner_core
  src/footprint.cpp
  src/primitives.cpp
  src/swept.cpp
  src/lattice.cpp
  src/compression.cpp
  src/route.cpp)
target_include_directories(mdp_planner_core PUBLIC include)

add_executable(mdp_plan_cli app/plan_cli.cpp)
target_link_libraries(mdp_plan_cli PRIVATE mdp_planner_core)

if(GTest_FOUND)
  add_executable(mdp_planner_core_tests
    test/test_primitives.cpp
    test/test_collision.cpp
    test/test_lattice.cpp
    test/test_route.cpp)
  target_link_libraries(mdp_planner_core_tests PRIVATE mdp_planner_core GTest::gtest GTest::gtest_main)
  gtest_discover_tests(mdp_planner_core_tests)
endif()
