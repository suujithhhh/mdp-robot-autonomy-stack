add_library(stm32_bridge_core src/bridge_core.cpp)
target_include_directories(stm32_bridge_core PUBLIC include)
target_link_libraries(stm32_bridge_core PUBLIC stm32_protocol)

if(GTest_FOUND)
  add_executable(stm32_bridge_tests test/test_bridge_core.cpp)
  target_link_libraries(stm32_bridge_tests PRIVATE stm32_bridge_core virtual_stm32 GTest::gtest GTest::gtest_main)
  gtest_discover_tests(stm32_bridge_tests)
endif()
