cmake_minimum_required(VERSION 3.8)
project(torpedo_control_v2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(torpedo_control_node_v2
  src/actuator_mixer.cpp
  src/control_core.cpp
  src/default_control_config.cpp
  src/keyboard_controller.cpp
  src/keyboard_input.cpp
  src/png_controller.cpp
  src/ros_interface.cpp
  src/roll_controller.cpp
  src/simple_tracking_controller.cpp
  src/torpedo_control_node.cpp
  src/util/tick_tock.cpp
  src/util/vector3.cpp
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_sources(torpedo_control_node_v2 PRIVATE src/cli_display.cpp)
endif()

target_include_directories(torpedo_control_node_v2 PUBLIC include)
target_compile_features(torpedo_control_node_v2 PUBLIC cxx_std_17)

ament_target_dependencies(torpedo_control_node_v2
  nav_msgs
  geometry_msgs
  rclcpp
  std_msgs
  tf2_ros
)

install(TARGETS torpedo_control_node_v2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
