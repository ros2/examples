cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_remapping)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(remap_node_constructor node_constructor.cpp)
ament_target_dependencies(remap_node_constructor rclcpp std_msgs)

add_executable(remap_rclcpp_init rclcpp_init.cpp)
ament_target_dependencies(remap_rclcpp_init rclcpp std_msgs)

install(TARGETS
  remap_node_constructor
  remap_rclcpp_init
  DESTINATION lib/${PROJECT_NAME})

ament_package()
