add_library(mdp_mission src/mission.cpp)
target_include_directories(mdp_mission PUBLIC include)
target_link_libraries(mdp_mission PUBLIC mdp_planner_core stm32_bridge_core virtual_stm32)

add_executable(mdp_sim_mission app/sim_mission.cpp)
target_link_libraries(mdp_sim_mission PRIVATE mdp_mission)

if(GTest_FOUND)
  add_executable(mdp_mission_tests test/test_end_to_end.cpp)
  target_link_libraries(mdp_mission_tests PRIVATE mdp_mission GTest::gtest GTest::gtest_main)
  gtest_discover_tests(mdp_mission_tests)
endif()
