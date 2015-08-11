# This file provides the add_executable_for_each_rmw_impl function.
# The function takes similar arguments as add_executable, but several executables.
# It creates one executable of the name you provide, which uses the default rmw implementation.
# It also creates a version of your executable for each rmw implementation specifically.
# These executables will be the name you gave plus a suffix like '__<impl_name>'.
# This makes it easier to test the examples across different middleware implementations.

# Get the rmw implementations which are available.
get_available_rmw_implementations(middleware_implementations)
foreach(middleware_impl ${middleware_implementations})
  # Find package each of them.
  find_package("${middleware_impl}" REQUIRED)
endforeach()

function(add_executable_for_each_rmw_impl executable)
  # Build the executable for the default rmw implementation.
  add_executable(${executable} ${ARGN})
  ament_target_dependencies(${executable}
    "rclcpp"
    "rmw_implementation"
    "std_msgs"
    "example_interfaces"
  )

  install(TARGETS ${executable} DESTINATION bin)

  # Build an executable for each rmw implementation.
  foreach(middleware_impl_tmp ${middleware_implementations})
    add_executable(${executable}__${middleware_impl_tmp} ${ARGN})
    ament_target_dependencies(${executable}__${middleware_impl_tmp}
      "rclcpp"
      "${middleware_impl_tmp}"
      "std_msgs"
      "example_interfaces"
    )

    install(TARGETS ${executable}__${middleware_impl_tmp} DESTINATION bin)
  endforeach()

endfunction()
