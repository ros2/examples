cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_executor_comparison)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

add_executable(publisher_node publisher_node.cc)
ament_target_dependencies(publisher_node rclcpp std_msgs)
install(TARGETS publisher_node DESTINATION lib/simple_thread_tester)

add_executable(single_thread_subscriber_node single_thread_subscriber_node.cc)
ament_target_dependencies(single_thread_subscriber_node rclcpp std_msgs)
install(TARGETS single_thread_subscriber_node
        DESTINATION lib/simple_thread_tester)

add_executable(multi_thread_reentrant_subscriber_node
               multi_thread_reentrant_subscriber_node.cc)
ament_target_dependencies(multi_thread_reentrant_subscriber_node rclcpp
                          std_msgs)
install(TARGETS multi_thread_reentrant_subscriber_node
        DESTINATION lib/simple_thread_tester)

add_executable(multi_thread_mutually_exclusive_subscriber_node
               multi_thread_mutually_exclusive_subscriber_node.cc)
ament_target_dependencies(multi_thread_mutually_exclusive_subscriber_node
                          rclcpp std_msgs)
install(TARGETS multi_thread_mutually_exclusive_subscriber_node
        DESTINATION lib/simple_thread_tester)

ament_package()
