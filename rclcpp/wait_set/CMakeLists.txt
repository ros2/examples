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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(talker SHARED src/talker.cpp)
target_compile_definitions(talker PRIVATE WAIT_SET_DLL)
ament_target_dependencies(talker rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(
    talker
    PLUGIN "Talker"
    EXECUTABLE wait_set_talker)

add_library(listener SHARED src/listener.cpp)
target_compile_definitions(listener PRIVATE WAIT_SET_DLL)
ament_target_dependencies(listener rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(
    listener
    PLUGIN "Listener"
    EXECUTABLE wait_set_listener)

add_executable(wait_set src/wait_set.cpp)
ament_target_dependencies(wait_set example_interfaces rclcpp std_msgs)

add_executable(static_wait_set src/static_wait_set.cpp)
ament_target_dependencies(static_wait_set rclcpp std_msgs)

add_executable(thread_safe_wait_set src/thread_safe_wait_set.cpp)
ament_target_dependencies(thread_safe_wait_set example_interfaces rclcpp std_msgs)

add_executable(wait_set_topics_and_timer src/wait_set_topics_and_timer.cpp)
ament_target_dependencies(wait_set_topics_and_timer rclcpp std_msgs)

add_executable(wait_set_random_order src/wait_set_random_order.cpp)
ament_target_dependencies(wait_set_random_order rclcpp std_msgs)

add_executable(executor_random_order src/executor_random_order.cpp)
ament_target_dependencies(executor_random_order rclcpp std_msgs)

add_executable(wait_set_topics_with_different_rates src/wait_set_topics_with_different_rates.cpp)
ament_target_dependencies(wait_set_topics_with_different_rates rclcpp std_msgs)

add_executable(wait_set_composed src/wait_set_composed.cpp)
target_link_libraries(wait_set_composed talker listener)
ament_target_dependencies(wait_set_composed rclcpp)

install(TARGETS
    talker
    listener
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS
  wait_set
  static_wait_set
  thread_safe_wait_set
  wait_set_topics_and_timer
  wait_set_random_order
  executor_random_order
  wait_set_topics_with_different_rates
  wait_set_composed
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
