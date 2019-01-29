cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_parameters)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(node_with_parameters src/node_with_parameters.cpp)
ament_target_dependencies(node_with_parameters rclcpp)

install(TARGETS
  node_with_parameters
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
