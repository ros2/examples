cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_cbg_executor)

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

add_executable(
  ping
  src/ping.cpp
  src/examples_rclcpp_cbg_executor/ping_node.cpp
)
target_include_directories(ping PUBLIC include)
ament_target_dependencies(ping rclcpp std_msgs)

add_executable(
  pong
  src/pong.cpp
  src/examples_rclcpp_cbg_executor/pong_node.cpp
)
target_include_directories(pong PUBLIC include)
ament_target_dependencies(pong rclcpp std_msgs)

add_executable(
  ping_pong
  src/ping_pong.cpp
  src/examples_rclcpp_cbg_executor/ping_node.cpp
  src/examples_rclcpp_cbg_executor/pong_node.cpp
)
target_include_directories(ping_pong PUBLIC include)
ament_target_dependencies(ping_pong rclcpp std_msgs)

install(TARGETS ping pong ping_pong
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp std_msgs)
ament_package()
