cmake_minimum_required(VERSION 3.16)
project(mdp_ros2_stack LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# -ffp-contract=off: no fused multiply-add, so C++ floating-point results match the
# reference Python planner operation for operation.
add_compile_options(-Wall -Wextra -ffp-contract=off)

option(MDP_SANITIZE "Build with AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
if(MDP_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

enable_testing()
find_package(GTest QUIET)
include(GoogleTest)
add_subdirectory(src/mdp_planner_core)
add_subdirectory(src/stm32_protocol)
add_subdirectory(src/virtual_stm32)
add_subdirectory(src/stm32_bridge)
add_subdirectory(src/mdp_mission)
