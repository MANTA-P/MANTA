cmake_minimum_required(VERSION 3.16)

if(ESP_PLATFORM)
  include($ENV{IDF_PATH}/tools/cmake/project.cmake)
  project(esp32_bridge)
  return()
endif()

project(esp32_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(esp32_ros_uart_tx_node
  src/ros_uart_tx_node.cpp
  src/packet_codec.cpp
  src/uart_port.cpp
)
target_include_directories(esp32_ros_uart_tx_node PRIVATE include)
ament_target_dependencies(esp32_ros_uart_tx_node
  rclcpp
  nav_msgs
  std_msgs
)

install(TARGETS esp32_ros_uart_tx_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_packet_codec
    test/test_packet_codec.cpp
    src/packet_codec.cpp
  )
  target_include_directories(test_packet_codec PRIVATE include)
endif()

ament_package()
