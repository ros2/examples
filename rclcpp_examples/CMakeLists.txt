cmake_minimum_required(VERSION 2.8.3)

project(rclcpp_examples)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_interfaces REQUIRED)

ament_package()

# This file provides the add_executable_for_each_rmw_impl function.
include(cmake/add_executable_for_each_rmw_impl.cmake)

##
## Examples of Publish/Subscribe with Topics
##

add_executable_for_each_rmw_impl(talker src/topics/talker.cpp)
add_executable_for_each_rmw_impl(listener src/topics/listener.cpp)

## Example of subscribe with topics and customized executor
add_executable_for_each_rmw_impl(listener_memory src/topics/listener_memory.cpp)

##
## Examples of Request/Response with Services
##

add_executable_for_each_rmw_impl(add_two_ints_client src/services/add_two_ints_client.cpp)
add_executable_for_each_rmw_impl(add_two_ints_client_async
  src/services/add_two_ints_client_async.cpp)
add_executable_for_each_rmw_impl(add_two_ints_server src/services/add_two_ints_server.cpp)

##
## Examples of Parameters with Asynchronous and Synchronous
##

add_executable_for_each_rmw_impl(list_parameters src/parameters/list_parameters.cpp)
add_executable_for_each_rmw_impl(list_parameters_async src/parameters/list_parameters_async.cpp)
add_executable_for_each_rmw_impl(parameter_events src/parameters/parameter_events.cpp)
add_executable_for_each_rmw_impl(parameter_events_async src/parameters/parameter_events_async.cpp)
add_executable_for_each_rmw_impl(set_and_get_parameters src/parameters/set_and_get_parameters.cpp)
add_executable_for_each_rmw_impl(set_and_get_parameters_async
  src/parameters/set_and_get_parameters_async.cpp)
