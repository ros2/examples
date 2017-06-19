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

add_executable(${PROJECT_NAME}_lambda lambda.cpp)
ament_target_dependencies(${PROJECT_NAME}_lambda rclcpp std_msgs)

add_executable(${PROJECT_NAME}_member_function member_function.cpp)
ament_target_dependencies(${PROJECT_NAME}_member_function rclcpp std_msgs)

add_executable(${PROJECT_NAME}_not_composable not_composable.cpp)
ament_target_dependencies(${PROJECT_NAME}_not_composable rclcpp std_msgs)

install(TARGETS
  ${PROJECT_NAME}_lambda
  ${PROJECT_NAME}_member_function
  ${PROJECT_NAME}_not_composable
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
