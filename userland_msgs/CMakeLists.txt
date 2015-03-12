cmake_minimum_required(VERSION 2.8.3)

project(userland_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${rosidl_default_generators_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  DEPENDENCIES builtin_msgs
)

ament_package()
