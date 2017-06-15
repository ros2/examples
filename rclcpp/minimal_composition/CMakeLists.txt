cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_composition)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_nodes SHARED
            src/publisher_node.cpp
            src/subscriber_node.cpp)
target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(${PROJECT_NAME}_nodes rclcpp std_msgs class_loader)

add_executable(${PROJECT_NAME}_publisher src/standalone_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_publisher ${PROJECT_NAME}_nodes)
ament_target_dependencies(${PROJECT_NAME}_publisher
  rclcpp ${PROJECT_NAME}_nodes)

add_executable(${PROJECT_NAME}_subscriber src/standalone_subscriber.cpp)
target_link_libraries(${PROJECT_NAME}_subscriber ${PROJECT_NAME}_nodes)
ament_target_dependencies(${PROJECT_NAME}_subscriber
  rclcpp ${PROJECT_NAME}_nodes)

add_executable(${PROJECT_NAME}_composed src/composed.cpp)
target_link_libraries(${PROJECT_NAME}_composed ${PROJECT_NAME}_nodes)
ament_target_dependencies(${PROJECT_NAME}_composed rclcpp class_loader)

install(TARGETS
  ${PROJECT_NAME}_nodes
  ${PROJECT_NAME}_publisher
  ${PROJECT_NAME}_subscriber
  ${PROJECT_NAME}_composed
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
