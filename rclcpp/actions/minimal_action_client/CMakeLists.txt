cmake_minimum_required(VERSION 3.20)
project(examples_rclcpp_minimal_action_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(action_client_member_functions member_functions.cpp)
target_link_libraries(action_client_member_functions PRIVATE
  example_interfaces::example_interfaces
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ament_cmake_ros_core::ament_ros_defaults)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  action_client_member_functions
  DESTINATION lib/${PROJECT_NAME})

ament_package()
