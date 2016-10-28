cmake_minimum_required(VERSION 2.8.3)

project(rcl_examples)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
#find_package(rmw_implementation REQUIRED)
#force opensplice implementation
find_package(rmw_opensplice_cpp REQUIRED)
#find_package(rmw_fastrtps_cpp REQUIRED)

ament_package()

##
## Examples of Publish/Subscribe with Topics
##

add_executable(get_node_names src/get_node_names.cpp)
ament_target_dependencies(get_node_names
  "rcl"
  "rmw_opensplice_cpp"
#  "rmw_fastrtps_cpp"

)

install(TARGETS get_node_names DESTINATION bin)
