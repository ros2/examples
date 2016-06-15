cmake_minimum_required(VERSION 3.5)

project(rclc_examples)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclc REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)

ament_package()

##
## Examples of Publish/Subscribe with Topics
##

add_executable(talker src/talker.c)
ament_target_dependencies(talker
  "rclc"
  "rmw_implementation"
  "std_msgs"
)

add_executable(listener src/listener.c)
ament_target_dependencies(listener
  "rclc"
  "rmw_implementation"
  "std_msgs"
)

install(TARGETS talker listener DESTINATION bin)
