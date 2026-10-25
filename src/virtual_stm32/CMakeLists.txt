add_library(virtual_stm32 src/virtual_stm.cpp)
target_include_directories(virtual_stm32 PUBLIC include)
target_link_libraries(virtual_stm32 PUBLIC stm32_protocol)

if(UNIX)
  add_executable(virtual_stm32_pty app/virtual_stm32_pty.cpp)
  target_link_libraries(virtual_stm32_pty PRIVATE virtual_stm32)
endif()

if(GTest_FOUND)
  add_executable(virtual_stm32_tests test/test_virtual_stm.cpp)
  target_link_libraries(virtual_stm32_tests PRIVATE virtual_stm32 GTest::gtest GTest::gtest_main)
  gtest_discover_tests(virtual_stm32_tests)
endif()
