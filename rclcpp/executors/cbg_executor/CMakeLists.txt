cmake_minimum_required(VERSION 3.20)
project(examples_rclcpp_cbg_executor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
target_link_libraries(ping PRIVATE rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(
  pong
  src/pong.cpp
  src/examples_rclcpp_cbg_executor/pong_node.cpp
)
target_include_directories(pong PUBLIC include)
target_link_libraries(pong PRIVATE rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(
  ping_pong
  src/ping_pong.cpp
  src/examples_rclcpp_cbg_executor/ping_node.cpp
  src/examples_rclcpp_cbg_executor/pong_node.cpp
)
target_include_directories(ping_pong PUBLIC include)
target_link_libraries(ping_pong PRIVATE rclcpp::rclcpp ${std_msgs_TARGETS})

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
