cmake_minimum_required(VERSION 3.8)
project(bluerov_integration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(dave_interfaces REQUIRED)

add_library(bluerov_common
  src/common/data_hub.cpp
)
target_include_directories(bluerov_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bluerov_common
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  dave_interfaces
)
target_link_libraries(bluerov_common Threads::Threads)

add_library(bluerov_path_follower
  src/integration/path_follower.cpp
)
target_include_directories(bluerov_path_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bluerov_path_follower
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  dave_interfaces
)

add_library(bluerov_team_min
  src/team_min/astar_planner.cpp
  src/team_min/dynamic_vo_planner.cpp
  src/team_min/planning_core.cpp
  src/team_min/planning_module.cpp
  src/team_min/rviz_visualizer.cpp
)
target_include_directories(bluerov_team_min PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bluerov_team_min
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  dave_interfaces
)
target_link_libraries(bluerov_team_min Threads::Threads)

add_library(ppid_log_plot_library
  src/team_byung/ppid_log_plot.cpp
)
target_compile_definitions(ppid_log_plot_library PRIVATE
  PPID_LOG_PLOT_LIBRARY
)
target_include_directories(ppid_log_plot_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(bluerov_team_byung
  src/team_byung/ppid_controller.cpp
  src/team_byung/ppid_logger.cpp
  src/team_byung/control_module.cpp
)
target_include_directories(bluerov_team_byung PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bluerov_team_byung
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  dave_interfaces
)
target_link_libraries(bluerov_team_byung
  ppid_log_plot_library
  Threads::Threads
)

add_executable(bluerov_integration_node
  src/bluerov_integration_node.cpp
)
target_include_directories(bluerov_integration_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(bluerov_integration_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  dave_interfaces
)
target_link_libraries(bluerov_integration_node
  bluerov_common
  bluerov_path_follower
  bluerov_team_min
  bluerov_team_byung
)

add_executable(target_position_input_node
  src/team_byung/target_position_input_node.cpp
)
ament_target_dependencies(target_position_input_node
  rclcpp
  geometry_msgs
)

add_executable(ppid_log_plot
  src/team_byung/ppid_log_plot.cpp
)
target_include_directories(ppid_log_plot PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(
  TARGETS
    bluerov_common
    bluerov_path_follower
    bluerov_team_min
    ppid_log_plot_library
    bluerov_team_byung
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS
    bluerov_integration_node
    target_position_input_node
    ppid_log_plot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
