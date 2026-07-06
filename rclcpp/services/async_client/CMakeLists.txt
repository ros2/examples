cmake_minimum_required(VERSION 3.20)
project(examples_rclcpp_async_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(client_main main.cpp)
target_link_libraries(client_main PRIVATE rclcpp::rclcpp example_interfaces::example_interfaces ament_cmake_ros_core::ament_ros_defaults)

install(TARGETS client_main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
