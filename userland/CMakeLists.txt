cmake_minimum_required(VERSION 2.8.3)

project(userland)

set(CMAKE_CXX_FLAGS "-std=c++0x -Wall")

find_package(ros_middleware_interface REQUIRED)
find_package(ros_dds_connext_dynamic REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_package()

include_directories(${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
add_definitions(${rclcpp_DEFINITIONS})
add_executable(userland src/main.cpp)
target_link_libraries(userland ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

install(
  TARGETS userland
  DESTINATION bin
)
