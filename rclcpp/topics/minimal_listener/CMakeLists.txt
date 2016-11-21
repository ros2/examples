cmake_minimum_required(VERSION 3.5)
project(minimal_listener)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
if (NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

add_executable(listener_lambda listener_lambda.cpp)
ament_target_dependencies(listener_lambda rclcpp std_msgs)

add_executable(listener_member_function listener_member_function.cpp)
ament_target_dependencies(listener_member_function rclcpp std_msgs)

add_executable(listener_without_subclass listener_without_subclass.cpp)
ament_target_dependencies(listener_without_subclass rclcpp std_msgs)

install(TARGETS listener_lambda
                listener_member_function
                listener_without_subclass
        DESTINATION bin)
ament_package()
