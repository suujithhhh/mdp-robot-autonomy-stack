add_library(can_bus src/frame.cpp src/bus.cpp src/robot_can.cpp)
target_include_directories(can_bus PUBLIC include)
target_link_libraries(can_bus PUBLIC stm32_protocol)

if(GTest_FOUND)
  add_executable(can_bus_tests test/test_frame.cpp test/test_bus.cpp test/test_robot_can.cpp)
  target_link_libraries(can_bus_tests PRIVATE can_bus GTest::gtest GTest::gtest_main)
  gtest_discover_tests(can_bus_tests)
endif()
