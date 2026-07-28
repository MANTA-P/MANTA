cmake_minimum_required(VERSION 3.8)
project(torpedo_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  torpedo_manual_control_node
  src/torpedo_manual_control_node.cpp
)

target_compile_features(torpedo_manual_control_node PUBLIC cxx_std_17)

ament_target_dependencies(
  torpedo_manual_control_node
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

# --- [v2 자동 추적 노드 추가] ---
add_executable(
  torpedo_manual_control_node_v2
  src/torpedo_manual_control_node_v2.cpp
)

target_compile_features(torpedo_manual_control_node_v2 PUBLIC cxx_std_17)

ament_target_dependencies(
  torpedo_manual_control_node_v2
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  TARGETS 
    torpedo_manual_control_node
    torpedo_manual_control_node_v2
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
