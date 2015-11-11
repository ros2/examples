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


add_executable_for_each_rmw_implementations(imu_listener
  src/topics/imu_listener.cpp
  TARGET_DEPENDENCIES ${target_dependencies} sensor_msgs INSTALL)

add_executable_for_each_rmw_implementations(allocator_example
  src/topics/allocator_example.cpp
  TARGET_DEPENDENCIES ${target_dependencies} INSTALL)


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

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  if(NOT WIN32)
    set(test_executable_extension "")
    set(test_executable_subfolder "")
  else()
    set(test_executable_extension ".exe")
    set(test_executable_subfolder "/${CMAKE_BUILD_TYPE}")
    if("${test_executable_subfolder}" STREQUAL "/")
      set(test_executable_subfolder "/Debug")
    endif()
  endif()

  # get the rmw implementations ahead of time
  get_available_rmw_implementations(middleware_implementations)
  foreach(middleware_impl ${middleware_implementations})
    find_package("${middleware_impl}" REQUIRED)
  endforeach()

  foreach(example_executable list_parameters_async list_parameters parameter_events_async parameter_events set_and_get_parameters_async set_and_get_parameters)
    foreach(middleware_impl ${middleware_implementations})
      # get typesupport of rmw implementation to include / link against the corresponding interfaces
      get_rmw_typesupport(typesupport_impl "${middleware_impl}")

      set(RCLCPP_EXAMPLES_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/${example_executable}__${middleware_impl}${test_executable_extension}")

      set(RCLCPP_EXAMPLES_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/${example_executable}.txt")

      configure_file(
        test/test_single_executable_example.py.in
        test_${example_executable}__${middleware_impl}.py
        @ONLY
      )

      ament_add_nose_test(test_example_${example_executable}__${middleware_impl} "${CMAKE_CURRENT_BINARY_DIR}/test_${example_executable}__${middleware_impl}.py" TIMEOUT 30)
      set_tests_properties(test_example_${example_executable}__${middleware_impl} PROPERTIES DEPENDS "test_example_${example_executable}__${middleware_impl} test_example_${example_executable}__${middleware_impl}")
    endforeach()
  endforeach()
endif()

ament_package()
