cmake_minimum_required(VERSION 2.8.3)

project(simple_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AllPrimitiveTypes.msg"
  "msg/Int32.msg"
  "msg/Intraprocess.msg"  
  "msg/Uint32.msg"
)

ament_package()
