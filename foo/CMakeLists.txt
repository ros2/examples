cmake_minimum_required(VERSION 2.8.3)

project(foo)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

set(_dds_vendor "$ENV{ROS_DDS_IMPLEMENTATION}")
if("${_dds_vendor}" STREQUAL "connext")
  find_package(ndds_cpp REQUIRED)
  set(CONNEXT_INCLUDE_DIRS ${ndds_cpp_INCLUDE_DIRS})
  set(CONNEXT_DEFINITIONS ${ndds_cpp_DEFINITIONS})
elseif("${_dds_vendor}" STREQUAL "opensplice")
  find_package(opensplice REQUIRED COMPONENTS CXX)
else()
  message(FATAL_ERROR "Set the environment variable 'ROS_DDS_IMPLEMENTATION' to either 'connext' or 'opensplice'.")
endif()

ament_package()

include_directories(include ${std_msgs_INCLUDE_DIRS} ${CONNEXT_INCLUDE_DIRS} ${OPENSPLICE_INCLUDE_DIRS})
add_definitions(${CONNEXT_DEFINITIONS})
add_executable(foo src/main.cpp)
message("std_msgs_LIBRARIES ${std_msgs_LIBRARIES}")
target_link_libraries(foo ${std_msgs_LIBRARIES})

install(
  TARGETS foo
  DESTINATION bin
)
