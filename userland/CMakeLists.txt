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

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${rclcpp_LIBRARIES} ${simple_msgs_LIBRARIES})

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${rclcpp_LIBRARIES} ${simple_msgs_LIBRARIES})

add_executable(prototype_intraprocess src/prototype_intraprocesses.cpp)
target_link_libraries(prototype_intraprocess ${rclcpp_LIBRARIES} ${simple_msgs_LIBRARIES})

install(
  TARGETS publisher subscriber prototype_intraprocess
  DESTINATION bin
)
