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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

set(target_dependencies
  "rclcpp"
  "std_msgs"
  "example_interfaces")

##
## Examples of Publish/Subscribe with Topics
##

add_executable_for_each_rmw_implementations(talker
  src/topics/talker.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(listener
  src/topics/listener.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(listener_best_effort
  src/topics/listener_best_effort.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)


## Example of publish/subscribe with topics and custom memory strategies for executor, subscriber
add_executable_for_each_rmw_implementations(listener_single_msg
  src/topics/listener_single_msg.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(listener_memory
  src/topics/listener_memory.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(talker_memory
  src/topics/talker_memory.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)

add_executable_for_each_rmw_implementations(imu_listener
  src/topics/imu_listener.cpp
  TARGET_DEPENDENCIES ${target_dependencies} sensor_msgs INSTALL)


##
## Examples of Request/Response with Services
##

add_executable_for_each_rmw_implementations(add_two_ints_client
  src/services/add_two_ints_client.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(add_two_ints_client_async
  src/services/add_two_ints_client_async.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(add_two_ints_server
  src/services/add_two_ints_server.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)

##
## Examples of Parameters with Asynchronous and Synchronous
##

add_executable_for_each_rmw_implementations(list_parameters
  src/parameters/list_parameters.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(list_parameters_async
  src/parameters/list_parameters_async.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(parameter_events
  src/parameters/parameter_events.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(parameter_events_async
  src/parameters/parameter_events_async.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(set_and_get_parameters
  src/parameters/set_and_get_parameters.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
add_executable_for_each_rmw_implementations(set_and_get_parameters_async
  src/parameters/set_and_get_parameters_async.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)
