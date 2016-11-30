cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_client)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()
find_package(ament_cmake REQUIRED)
find_package(examples_rclcpp_minimal_service REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME}_not_composable not_composable.cpp)
ament_target_dependencies(${PROJECT_NAME}_not_composable
                          rclcpp examples_rclcpp_minimal_service)

install(TARGETS ${PROJECT_NAME}_not_composable
        DESTINATION bin)

ament_package()
