cmake_minimum_required(VERSION 3.20)
project(examples_rclcpp_minimal_composition)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(composition_nodes SHARED
            src/publisher_node.cpp
            src/subscriber_node.cpp)
target_compile_definitions(composition_nodes
  PRIVATE "MINIMAL_COMPOSITION_DLL")
target_link_libraries(composition_nodes PUBLIC rclcpp::rclcpp rclcpp_components::component ${std_msgs_TARGETS})

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(composition_publisher src/standalone_publisher.cpp)
target_link_libraries(composition_publisher PRIVATE composition_nodes rclcpp::rclcpp)

add_executable(composition_subscriber src/standalone_subscriber.cpp)
target_link_libraries(composition_subscriber PRIVATE composition_nodes rclcpp::rclcpp)

add_executable(composition_composed src/composed.cpp)
target_link_libraries(composition_composed PRIVATE composition_nodes rclcpp::rclcpp class_loader::class_loader)

install(TARGETS
  composition_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  composition_publisher
  composition_subscriber
  composition_composed
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
