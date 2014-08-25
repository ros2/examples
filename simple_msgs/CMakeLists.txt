cmake_minimum_required(VERSION 2.8.3)

project(simple_msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AllDynamicArrayTypes.msg"
  "msg/AllPrimitiveConstants.msg"
  "msg/AllPrimitiveTypes.msg"
  "msg/AllStaticArrayTypes.msg"
  "msg/Int32.msg"
  "msg/Intraprocess.msg"
  "msg/Nested.msg"
  "msg/String.msg"
  "msg/Uint32.msg"
)

ament_package()
