cmake_minimum_required(VERSION 2.8.3)

project(userland)

set(CMAKE_CXX_FLAGS "-std=c++0x -Wall")

find_package(ros_middleware_interface REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(simple_msgs REQUIRED)

ament_package()

include_directories(${rclcpp_INCLUDE_DIRS} ${simple_msgs_INCLUDE_DIRS})
add_definitions(${rclcpp_DEFINITIONS})
add_executable(userland src/publisher.cpp)
target_link_libraries(userland ${rclcpp_LIBRARIES} ${simple_msgs_LIBRARIES})

install(
  TARGETS userland
  DESTINATION bin
)
