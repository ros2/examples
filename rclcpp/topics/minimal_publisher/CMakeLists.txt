cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_publisher)

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

add_executable(publisher_lambda lambda.cpp)
ament_target_dependencies(publisher_lambda rclcpp std_msgs)

add_executable(publisher_member_function member_function.cpp)
ament_target_dependencies(publisher_member_function rclcpp std_msgs)

add_executable(publisher_not_composable not_composable.cpp)
ament_target_dependencies(publisher_not_composable rclcpp std_msgs)

install(TARGETS
  publisher_lambda
  publisher_member_function
  publisher_not_composable
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
