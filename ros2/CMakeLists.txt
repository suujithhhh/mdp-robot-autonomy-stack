cmake_minimum_required(VERSION 3.16)
project(mdp_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# No fused multiply-add: keep floating-point results identical to the reference Python planner.
add_compile_options(-Wall -Wextra -ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# The pure C++ libraries (no ROS dependency) live in ../src in the repository; CI copies them to
# ./core before running colcon, because colcon only sees the package directory.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/core)
  set(MDP_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/core)
else()
  set(MDP_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)
endif()

find_package(GTest QUIET)
include(GoogleTest)
add_subdirectory(${MDP_CORE_DIR}/mdp_planner_core core/mdp_planner_core)
add_subdirectory(${MDP_CORE_DIR}/stm32_protocol core/stm32_protocol)
add_subdirectory(${MDP_CORE_DIR}/virtual_stm32 core/virtual_stm32)
add_subdirectory(${MDP_CORE_DIR}/stm32_bridge core/stm32_bridge)
add_subdirectory(${MDP_CORE_DIR}/mdp_mission core/mdp_mission)

add_executable(stm32_bridge_node nodes/stm32_bridge_node.cpp)
ament_target_dependencies(stm32_bridge_node rclcpp std_msgs nav_msgs sensor_msgs diagnostic_msgs
                          geometry_msgs tf2_ros)
target_link_libraries(stm32_bridge_node stm32_bridge_core)

add_executable(mdp_planner_node nodes/mdp_planner_node.cpp)
ament_target_dependencies(mdp_planner_node rclcpp std_msgs std_srvs nav_msgs geometry_msgs
                          visualization_msgs)
target_link_libraries(mdp_planner_node mdp_planner_core)

add_executable(mission_node nodes/mission_node.cpp)
ament_target_dependencies(mission_node rclcpp std_msgs std_srvs)
target_link_libraries(mission_node stm32_protocol)

install(TARGETS stm32_bridge_node mdp_planner_node mission_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(UNIX)
  install(TARGETS virtual_stm32_pty DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
