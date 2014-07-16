cmake_minimum_required(VERSION 2.8.3)

project(foo)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

find_package(opensplice REQUIRED COMPONENTS CXX)

ament_package()

include_directories(include ${std_msgs_INCLUDE_DIRS} ${OPENSPLICE_INCLUDE_DIRS})
add_executable(foo src/main.cpp)
message("std_msgs_LIBRARIES ${std_msgs_LIBRARIES}")
target_link_libraries(foo ${std_msgs_LIBRARIES})

install(
  TARGETS foo
  DESTINATION bin
)
