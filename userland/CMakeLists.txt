cmake_minimum_required(VERSION 2.8.3)

project(userland)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(ros_middleware_interface REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_middleware_implementation REQUIRED)
find_package(simple_msgs REQUIRED)

ament_package()

include_directories(include)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher
  "rclcpp"
  "ros_middleware_implementation"
  "simple_msgs")

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber
  "rclcpp"
  "ros_middleware_implementation"
  "simple_msgs")

add_executable(prototype_intraprocess src/prototype_intraprocesses.cpp)
ament_target_dependencies(prototype_intraprocess
  "rclcpp"
  "ros_middleware_implementation"
  "simple_msgs")

install(
  TARGETS publisher subscriber prototype_intraprocess
  DESTINATION bin
)


# for testing generate binaries for each middleware implementation
get_available_ros_middleware_implementations(middleware_implementations)
foreach(middleware_implementation ${middleware_implementations})
  find_package("${middleware_implementation}" REQUIRED)

  add_executable(publisher__${middleware_implementation} src/publisher.cpp)
  ament_target_dependencies(publisher__${middleware_implementation}
    "rclcpp"
    "${middleware_implementation}"
    "simple_msgs")

  add_executable(subscriber__${middleware_implementation} src/subscriber.cpp)
  ament_target_dependencies(subscriber__${middleware_implementation}
    "rclcpp"
    "${middleware_implementation}"
    "simple_msgs")

  install(
    TARGETS publisher__${middleware_implementation} subscriber__${middleware_implementation}
    DESTINATION bin
  )
endforeach()
