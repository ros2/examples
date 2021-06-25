cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_wait_set)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(wait_set wait_set.cpp)
ament_target_dependencies(wait_set example_interfaces rclcpp std_msgs)

add_executable(static_wait_set static_wait_set.cpp)
ament_target_dependencies(static_wait_set rclcpp std_msgs)

add_executable(thread_safe_wait_set thread_safe_wait_set.cpp)
ament_target_dependencies(thread_safe_wait_set example_interfaces rclcpp std_msgs)

add_executable(waitset_example_main waitset_example_main.cpp)
ament_target_dependencies(waitset_example_main rclcpp std_msgs)

add_executable(waitset_example_main_loop waitset_example_main_loop.cpp)
ament_target_dependencies(waitset_example_main_loop rclcpp std_msgs)

add_executable(waitset_composition waitset_composition.cpp)
ament_target_dependencies(waitset_composition rclcpp std_msgs)

install(TARGETS
  wait_set
  static_wait_set
  thread_safe_wait_set
  waitset_example_main
  waitset_example_main_loop
  waitset_composition
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
