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

# Get the implementations ahead of time
get_available_ros_middleware_implementations(middleware_implementations)
foreach(middleware_implementation ${middleware_implementations})
  find_package("${middleware_implementation}" REQUIRED)
endforeach()

function(build_executable executable)
  # Build the normal executable
  add_executable(${executable} ${ARGN})
  ament_target_dependencies(${executable}
    "rclcpp"
    "ros_middleware_implementation"
    "simple_msgs")

  install(TARGETS ${executable} DESTINATION bin)

  # Build an executable for each ROS middleware implementation
  foreach(middleware_implementation ${middleware_implementations})
    add_executable(${executable}__${middleware_implementation} ${ARGN})
    ament_target_dependencies(${executable}__${middleware_implementation}
    "rclcpp"
    "${middleware_implementation}"
    "simple_msgs")

    install(TARGETS ${executable}__${middleware_implementation} DESTINATION bin)
  endforeach()

endfunction()

build_executable(talker src/ros1_like/talker.cpp)

build_executable(listener src/ros1_like/listener.cpp)

build_executable(publisher src/publisher.cpp)

build_executable(subscriber src/subscriber.cpp)

build_executable(prototype_intraprocess src/prototype_intraprocesses.cpp)
