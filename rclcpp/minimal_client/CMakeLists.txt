cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_client)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(test_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(client_main main.cpp)
ament_target_dependencies(client_main rclcpp example_interfaces)

add_executable(client_main_action main_action.cpp)
ament_target_dependencies(client_main_action rclcpp test_msgs)

install(TARGETS client_main
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS client_main_action
  DESTINATION lib/${PROJECT_NAME})

ament_package()
