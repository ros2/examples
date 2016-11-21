cmake_minimum_required(VERSION 3.5)
project(minimal_talker)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
if (NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

add_executable(talker_timer_lambda talker_timer_lambda.cpp)
ament_target_dependencies(talker_timer_lambda rclcpp std_msgs)

add_executable(talker_timer_member_function talker_timer_member_function.cpp)
ament_target_dependencies(talker_timer_member_function rclcpp std_msgs)

add_executable(talker_without_subclass talker_without_subclass.cpp)
ament_target_dependencies(talker_without_subclass rclcpp std_msgs)

install(TARGETS talker_timer_lambda
                talker_timer_member_function
                talker_without_subclass
        DESTINATION bin)
ament_package()
