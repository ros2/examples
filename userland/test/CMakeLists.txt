find_package(ament_cmake_nose REQUIRED)

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

foreach(middleware_impl_tmp ${middleware_implementations})
  # test publish / subscribe
  add_executable(test_publisher__${middleware_impl_tmp} "test_publisher.cpp")
  ament_target_dependencies(test_publisher__${middleware_impl_tmp}
    "rclcpp"
    "${middleware_impl_tmp}"
    "simple_msgs")

  add_executable(test_subscriber__${middleware_impl_tmp} "test_subscriber.cpp")
  ament_target_dependencies(test_subscriber__${middleware_impl_tmp}
    "rclcpp"
    "${middleware_impl_tmp}"
    "simple_msgs")

  set(TEST_PUBLISHER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/test_publisher__${middleware_impl_tmp}${test_executable_extension}")
  set(TEST_SUBSCRIBER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/test_subscriber__${middleware_impl_tmp}${test_executable_extension}")
  configure_file(
    test_publish_subscribe.py.in
    test_publish_subscribe__${middleware_impl_tmp}.py
    @ONLY
  )

  ament_add_nose_test(publish_subscribe__${middleware_impl_tmp} "${CMAKE_CURRENT_BINARY_DIR}/test_publish_subscribe__${middleware_impl_tmp}.py" TIMEOUT 60)
  set_tests_properties(
    publish_subscribe__${middleware_impl_tmp}
    PROPERTIES DEPENDS "test_publisher__${middleware_impl_tmp};test_subscriber__${middleware_impl_tmp}"
  )

  # skip Connext XTypes for now
  if(NOT "${middleware_impl_tmp} " STREQUAL "rmw_connext_xtypes_dynamic_cpp ")
  # test server / client
  add_executable(test_server__${middleware_impl_tmp} "test_server.cpp")
  ament_target_dependencies(test_server__${middleware_impl_tmp}
    "rclcpp"
    "${middleware_impl_tmp}"
    "userland_msgs")

  add_executable(test_client__${middleware_impl_tmp} "test_client.cpp")
  ament_target_dependencies(test_client__${middleware_impl_tmp}
    "rclcpp"
    "${middleware_impl_tmp}"
    "userland_msgs")

  if(NOT WIN32)
    set(test_executable_extension "")
  else()
    set(test_executable_extension ".exe")
  endif()
  set(TEST_SERVER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/test_server__${middleware_impl_tmp}${test_executable_extension}")
  set(TEST_CLIENT_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/test_client__${middleware_impl_tmp}${test_executable_extension}")
  configure_file(
    test_request_response.py.in
    test_request_response__${middleware_impl_tmp}.py
    @ONLY
  )

  ament_add_nose_test(request_response__${middleware_impl_tmp} "${CMAKE_CURRENT_BINARY_DIR}/test_request_response__${middleware_impl_tmp}.py" TIMEOUT 60)
  set_tests_properties(
    request_response__${middleware_impl_tmp}
    PROPERTIES DEPENDS "test_server__${middleware_impl_tmp};test_client__${middleware_impl_tmp}"
  )
  endif()
endforeach()
