cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_timer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(timer_lambda lambda.cpp)
ament_target_dependencies(timer_lambda rclcpp)

add_executable(timer_member_function member_function.cpp)
ament_target_dependencies(timer_member_function rclcpp)

install(TARGETS
  timer_lambda
  timer_member_function
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
