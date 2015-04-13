find_package(ament_cmake_nose REQUIRED)

foreach(middleware_impl_tmp ${middleware_implementations})

  add_executable(test_publisher__${middleware_impl_tmp} "test_publisher.cpp")
  ament_target_dependencies(test_publisher__${middleware_impl_tmp}
    "rcl_interfaces"
    "rclcpp"
    "${middleware_impl_tmp}"
    "simple_msgs")

  add_executable(test_subscriber__${middleware_impl_tmp} "test_subscriber.cpp")
  ament_target_dependencies(test_subscriber__${middleware_impl_tmp}
    "rcl_interfaces"
    "rclcpp"
    "${middleware_impl_tmp}"
    "simple_msgs")

  if(NOT WIN32)
    set(test_executable_extension "")
  else()
    set(test_executable_extension ".exe")
  endif()
  set(TEST_PUBLISHER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}/test_publisher__${middleware_impl_tmp}${test_executable_extension}")
  set(TEST_SUBSCRIBER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}/test_subscriber__${middleware_impl_tmp}${test_executable_extension}")
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
endforeach()
