cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_service)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME}_main main.cpp)
ament_target_dependencies(${PROJECT_NAME}_main rclcpp example_interfaces)

install(TARGETS ${PROJECT_NAME}_main
        DESTINATION bin)

ament_package()
