cmake_minimum_required(VERSION 2.8.3)

project(userland)

set(CMAKE_CXX_FLAGS "-std=c++0x -Wall")

find_package(ros_middleware_interface REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_middleware_implementation REQUIRED)
find_package(simple_msgs REQUIRED)

ament_package()

include_directories(${rclcpp_INCLUDE_DIRS} ${simple_msgs_INCLUDE_DIRS})
add_definitions(${rclcpp_DEFINITIONS})

add_executable(publisher src/publisher.cpp)
target_include_directories(publisher PUBLIC ${ros_middleware_implementation_INCLUDE_DIRS})
target_link_libraries(publisher ${rclcpp_LIBRARIES} ${ros_middleware_implementation_LIBRARIES} ${simple_msgs_LIBRARIES})

add_executable(subscriber src/subscriber.cpp)
target_include_directories(subscriber PUBLIC ${ros_middleware_implementation_INCLUDE_DIRS})
target_link_libraries(subscriber ${rclcpp_LIBRARIES} ${ros_middleware_implementation_LIBRARIES} ${simple_msgs_LIBRARIES})

add_executable(prototype_intraprocess src/prototype_intraprocesses.cpp)
target_link_libraries(prototype_intraprocess ${rclcpp_LIBRARIES} ${simple_msgs_LIBRARIES})

install(
  TARGETS publisher subscriber prototype_intraprocess
  DESTINATION bin
)


# for testing generate binaries for each middleware implementation
get_available_ros_middleware_implementations(middleware_implementations)
foreach(middleware_implementation ${middleware_implementations})
  find_package("${middleware_implementation}" REQUIRED)

  add_executable(publisher__${middleware_implementation} src/publisher.cpp)
  target_include_directories(publisher__${middleware_implementation} PUBLIC ${${middleware_implementation}_INCLUDE_DIRS})
  target_link_libraries(publisher__${middleware_implementation} ${rclcpp_LIBRARIES} ${${middleware_implementation}_LIBRARIES} ${simple_msgs_LIBRARIES})

  add_executable(subscriber__${middleware_implementation} src/subscriber.cpp)
  target_include_directories(subscriber__${middleware_implementation} PUBLIC ${${middleware_implementation}_INCLUDE_DIRS})
  target_link_libraries(subscriber__${middleware_implementation} ${rclcpp_LIBRARIES} ${${middleware_implementation}_LIBRARIES} ${simple_msgs_LIBRARIES})

  install(
    TARGETS publisher__${middleware_implementation} subscriber__${middleware_implementation}
    DESTINATION bin
  )
endforeach()
