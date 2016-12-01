cmake_minimum_required(VERSION 3.5)

project(rclcpp_examples)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

function(custom_executable subfolder target)
  add_executable(${target}${target_suffix} src/${subfolder}/${target}.cpp)
  # TODO(dhood): Allow dependencies to be listed in arbitrary order without affecting the selected
  # typesupport. See https://github.com/ros2/ros2/issues/253
  ament_target_dependencies(${target}${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs"
    "example_interfaces")
  install(TARGETS ${target}${target_suffix}
    DESTINATION bin)
endfunction()

macro(targets)
  if(NOT target_suffix STREQUAL "")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  # Examples of Publish/Subscribe with Topics
  custom_executable(topics talker)
  custom_executable(topics listener)
  custom_executable(topics listener_best_effort)
  custom_executable(topics imu_listener)
  ament_target_dependencies(imu_listener${target_suffix}
    "sensor_msgs")
  custom_executable(topics allocator_example)

  # Examples of Request/Response with Services
  custom_executable(services add_two_ints_client)
  custom_executable(services add_two_ints_client_async)
  custom_executable(services add_two_ints_server)

  # Examples of Parameters with Asynchronous and Synchronous
  custom_executable(parameters list_parameters)
  custom_executable(parameters list_parameters_async)
  custom_executable(parameters parameter_events)
  custom_executable(parameters parameter_events_async)
  custom_executable(parameters set_and_get_parameters)
  custom_executable(parameters set_and_get_parameters_async)
  custom_executable(parameters ros2param)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add each test case.  Multi-executable tests can be specified in
  # semicolon-separated strings, like  exe1;exe2.
  set(example_tests
    list_parameters_async
    list_parameters
    parameter_events_async
    parameter_events
    set_and_get_parameters_async
    set_and_get_parameters
    "talker:listener")
  set(service_example_tests
    "add_two_ints_server:add_two_ints_client"
    "add_two_ints_server:add_two_ints_client_async")

  macro(tests)
    set(example_tests_to_test ${example_tests})
    list(APPEND example_tests_to_test ${service_example_tests})

    foreach(example_test ${example_tests_to_test})
      string(REPLACE ":" ";" example_executables "${example_test}")
      set(RCLCPP_EXAMPLES_EXPECTED_OUTPUT "")
      foreach(executable ${example_executables})
        list(APPEND RCLCPP_EXAMPLES_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/${executable}")
      endforeach()

      set(RCLCPP_EXAMPLES_EXECUTABLE "")
      foreach(executable ${example_executables})
        list(APPEND RCLCPP_EXAMPLES_EXECUTABLE "$<TARGET_FILE:${executable}${target_suffix}>")
      endforeach()

      string(REPLACE ";" "_" exe_list_underscore "${example_executables}")
      configure_file(
        test/test_executables_example.py.in
        test_${exe_list_underscore}${target_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "test_${exe_list_underscore}${target_suffix}_$<CONFIGURATION>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}.py.configured"
      )

      ament_add_nose_test(test_example_${exe_list_underscore}${target_suffix}
        "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIGURATION>.py"
        TIMEOUT 30
        ENV RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation})
      foreach(executable ${example_executables})
        set_property(
          TEST test_example_${exe_list_underscore}${target_suffix}
          APPEND PROPERTY DEPENDS ${executable}${target_suffix})
      endforeach()
    endforeach()
  endmacro()

  call_for_each_rmw_implementation(tests)
endif()

ament_package()
