cmake_minimum_required(VERSION 2.8.3)

project(rclc_examples)

if(NOT WIN32)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)

ament_package()

##
## Examples of Publish/Subscribe with Topics
##

macro(example_targets)
  add_executable(int32_talker_c__${rmw_implementation} src/int32_talker.c)
  ament_target_dependencies(int32_talker_c__${rmw_implementation}
    "rcl"
    "${rmw_implementation}"
    "std_msgs"
  )

  add_executable(int32_listener_c__${rmw_implementation} src/int32_listener.c)
  ament_target_dependencies(int32_listener_c__${rmw_implementation}
    "rcl"
    "${rmw_implementation}"
    "std_msgs"
  )

  add_executable(talker_c__${rmw_implementation} src/talker.c)
  ament_target_dependencies(talker_c__${rmw_implementation}
    "rcl"
    "${rmw_implementation}"
    "std_msgs"
  )

  add_executable(listener_c__${rmw_implementation} src/listener.c)
  ament_target_dependencies(listener_c__${rmw_implementation}
    "rcl"
    "${rmw_implementation}"
    "std_msgs"
  )

  install(
    TARGETS
      int32_talker_c__${rmw_implementation}
      int32_listener_c__${rmw_implementation}
      talker_c__${rmw_implementation}
      listener_c__${rmw_implementation}
    DESTINATION bin
  )
endmacro()

call_for_each_rmw_implementation(example_targets)

