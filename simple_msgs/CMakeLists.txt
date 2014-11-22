cmake_minimum_required(VERSION 2.8.3)

project(simple_msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(ament_cmake REQUIRED)
find_package(builtin_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${rosidl_default_generators_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Time.msg"
  "msg/Header.msg"
  "msg/Quaternion32.msg"
  "msg/Imu32.msg"
  "msg/Vector3.msg"
  "msg/Vector3Int.msg"
  "msg/Vector3Float.msg"
  "msg/AllBuiltinTypes.msg"
  "msg/AllDynamicArrayTypes.msg"
  "msg/AllPrimitiveConstants.msg"
  "msg/AllPrimitiveTypes.msg"
  "msg/AllStaticArrayTypes.msg"
  "msg/Int32.msg"
  "msg/Intraprocess.msg"
  "msg/Nested.msg"
  "msg/String.msg"
  "msg/Uint32.msg"
  DEPENDENCIES builtin_msgs
)

ament_package()
