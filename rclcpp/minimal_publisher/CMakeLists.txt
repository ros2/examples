cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_publisher)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
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
  DESTINATION bin
)

ament_package()
