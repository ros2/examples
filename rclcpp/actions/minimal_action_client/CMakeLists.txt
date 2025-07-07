cmake_minimum_required(VERSION 3.20)
project(examples_rclcpp_minimal_action_client)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(action_client_member_functions member_functions.cpp)
target_link_libraries(action_client_member_functions PRIVATE
  ${example_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action)

add_executable(action_client_not_composable not_composable.cpp)
target_link_libraries(action_client_not_composable PRIVATE
  ${example_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action)

add_executable(action_client_not_composable_with_cancel not_composable_with_cancel.cpp)
target_link_libraries(action_client_not_composable_with_cancel PRIVATE
  ${example_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action)

add_executable(action_client_not_composable_with_feedback not_composable_with_feedback.cpp)
target_link_libraries(action_client_not_composable_with_feedback PRIVATE
  ${example_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action)

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
