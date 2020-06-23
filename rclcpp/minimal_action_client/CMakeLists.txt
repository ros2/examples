cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_action_client)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(action_client_member_functions member_functions.cpp)
ament_target_dependencies(action_client_member_functions
  "rclcpp"
  "rclcpp_action"
  "example_interfaces")

add_executable(action_client_not_composable not_composable.cpp)
ament_target_dependencies(action_client_not_composable
  "rclcpp"
  "rclcpp_action"
  "example_interfaces")

add_executable(action_client_not_composable_with_cancel not_composable_with_cancel.cpp)
ament_target_dependencies(action_client_not_composable_with_cancel
  "rclcpp"
  "rclcpp_action"
  "example_interfaces")

add_executable(action_client_not_composable_with_feedback not_composable_with_feedback.cpp)
ament_target_dependencies(action_client_not_composable_with_feedback
  "rclcpp"
  "rclcpp_action"
  "example_interfaces")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  action_client_member_functions
  action_client_not_composable
  action_client_not_composable_with_cancel
  action_client_not_composable_with_feedback
  DESTINATION lib/${PROJECT_NAME})

ament_package()
