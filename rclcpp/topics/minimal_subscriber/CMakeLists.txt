cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_subscriber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(subscriber_lambda lambda.cpp)
ament_target_dependencies(subscriber_lambda rclcpp std_msgs)

add_executable(subscriber_member_function member_function.cpp)
ament_target_dependencies(subscriber_member_function rclcpp std_msgs)

add_executable(subscriber_member_function_with_topic_statistics member_function_with_topic_statistics.cpp)
ament_target_dependencies(subscriber_member_function_with_topic_statistics rclcpp std_msgs)

add_executable(subscriber_member_function_with_type_adapter member_function_with_type_adapter.cpp)
ament_target_dependencies(subscriber_member_function_with_type_adapter rclcpp std_msgs)

add_executable(subscriber_member_function_with_unique_network_flow_endpoints member_function_with_unique_network_flow_endpoints.cpp)
ament_target_dependencies(subscriber_member_function_with_unique_network_flow_endpoints rclcpp std_msgs)

add_executable(subscriber_not_composable not_composable.cpp)
ament_target_dependencies(subscriber_not_composable rclcpp std_msgs)

add_executable(subscriber_content_filtering content_filtering.cpp)
ament_target_dependencies(subscriber_content_filtering rclcpp std_msgs)

add_library(wait_set_subscriber_library SHARED
    wait_set_subscriber.cpp
    static_wait_set_subscriber.cpp
    time_triggered_wait_set_subscriber.cpp)
ament_target_dependencies(wait_set_subscriber_library rclcpp rclcpp_components std_msgs)

rclcpp_components_register_node(wait_set_subscriber_library
    PLUGIN "WaitSetSubscriber"
    EXECUTABLE wait_set_subscriber)

rclcpp_components_register_node(wait_set_subscriber_library
    PLUGIN "StaticWaitSetSubscriber"
    EXECUTABLE static_wait_set_subscriber)

rclcpp_components_register_node(wait_set_subscriber_library
    PLUGIN "TimeTriggeredWaitSetSubscriber"
    EXECUTABLE time_triggered_wait_set_subscriber)

install(TARGETS
    wait_set_subscriber_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(TARGETS
  subscriber_lambda
  subscriber_member_function
  subscriber_member_function_with_topic_statistics
  subscriber_member_function_with_type_adapter
  subscriber_member_function_with_unique_network_flow_endpoints
  subscriber_not_composable
  subscriber_content_filtering
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
