cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_subscriber)

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

add_executable(subscriber_lambda lambda.cpp)
ament_target_dependencies(subscriber_lambda rclcpp std_msgs)

add_executable(subscriber_member_function member_function.cpp)
ament_target_dependencies(subscriber_member_function rclcpp std_msgs)

add_executable(subscriber_not_composable not_composable.cpp)
ament_target_dependencies(subscriber_not_composable rclcpp std_msgs)

install(TARGETS
  subscriber_lambda
  subscriber_member_function
  subscriber_not_composable
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
