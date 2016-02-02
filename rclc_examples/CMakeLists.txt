cmake_minimum_required(VERSION 2.8.3)

project(rclc_examples)

if(NOT WIN32)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)

ament_package()

##
## Examples of Publish/Subscribe with Topics
##

add_executable(int32_talker_c src/int32_talker.c)
ament_target_dependencies(int32_talker_c
  "rcl"
  # "rmw_implementation"
  "rmw_opensplice_cpp"
  "std_msgs"
)

add_executable(int32_listener_c src/int32_listener.c)
ament_target_dependencies(int32_listener_c
  "rcl"
  # "rmw_implementation"
  "rmw_opensplice_cpp"
  "std_msgs"
)

add_executable(talker_c src/talker.c)
ament_target_dependencies(talker_c
  "rcl"
  # "rmw_implementation"
  "rmw_opensplice_cpp"
  "std_msgs"
)

add_executable(listener_c src/listener.c)
ament_target_dependencies(listener_c
  "rcl"
  # "rmw_implementation"
  "rmw_opensplice_cpp"
  "std_msgs"
)

install(TARGETS int32_talker_c int32_listener_c talker_c listener_c DESTINATION bin)
