cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_timer)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME}_lambda lambda.cpp)
ament_target_dependencies(${PROJECT_NAME}_lambda rclcpp)

add_executable(${PROJECT_NAME}_member_function member_function.cpp)
ament_target_dependencies(${PROJECT_NAME}_member_function rclcpp)

install(TARGETS
  ${PROJECT_NAME}_lambda
  ${PROJECT_NAME}_member_function
  DESTINATION bin
)

ament_package()
