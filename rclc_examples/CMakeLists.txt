cmake_minimum_required(VERSION 3.5)

project(rclc_examples)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclc REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)

ament_package()

##
## Examples of Publish/Subscribe with Topics
##
macro(targets)
  if(NOT target_suffix STREQUAL "")
    get_rcl_information("${rmw_implementation}" "rcl${target_suffix}")
  endif()

  add_executable(talker_c${target_suffix}
    src/talker.c)
  ament_target_dependencies(talker_c${target_suffix}
    "rclc${target_suffix}"
    "rmw_implementation"
    "std_msgs")
  install(TARGETS talker_c${target_suffix}
    DESTINATION bin)

  add_executable(listener_c${target_suffix}
    src/listener.c)
  ament_target_dependencies(listener_c${target_suffix}
    "rclc${target_suffix}"
    "rmw_implementation"
    "std_msgs")
  install(TARGETS listener_c${target_suffix}
    DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)
