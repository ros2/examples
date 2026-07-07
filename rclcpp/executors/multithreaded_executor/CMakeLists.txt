cmake_minimum_required(VERSION 3.20)
project(examples_rclcpp_multithreaded_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(multithreaded_executor multithreaded_executor.cpp)
target_link_libraries(multithreaded_executor PRIVATE rclcpp::rclcpp std_msgs::std_msgs ament_cmake_ros_core::ament_ros_defaults)

install(TARGETS
  multithreaded_executor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
