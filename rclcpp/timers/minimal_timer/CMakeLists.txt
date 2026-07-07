cmake_minimum_required(VERSION 3.20)
project(examples_rclcpp_minimal_timer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(timer_lambda lambda.cpp)
target_link_libraries(timer_lambda PRIVATE rclcpp::rclcpp ament_cmake_ros_core::ament_ros_defaults)

add_executable(timer_member_function member_function.cpp)
target_link_libraries(timer_member_function PRIVATE rclcpp::rclcpp ament_cmake_ros_core::ament_ros_defaults)

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
