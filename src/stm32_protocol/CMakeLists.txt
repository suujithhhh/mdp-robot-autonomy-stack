add_library(stm32_protocol src/protocol.cpp)
target_include_directories(stm32_protocol PUBLIC include)
target_link_libraries(stm32_protocol PUBLIC mdp_planner_core)

if(GTest_FOUND)
  add_executable(stm32_protocol_tests test/test_protocol.cpp)
  target_link_libraries(stm32_protocol_tests PRIVATE stm32_protocol GTest::gtest GTest::gtest_main)
  gtest_discover_tests(stm32_protocol_tests)
endif()
