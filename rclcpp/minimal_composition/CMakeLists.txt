cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_composition)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_nodes SHARED publisher_node.cpp subscriber_node.cpp)
target_compile_definitions(${PROJECT_NAME}_nodes PRIVATE "COMPOSITION_DLL")
ament_target_dependencies(${PROJECT_NAME}_nodes rclcpp std_msgs class_loader)

add_executable(${PROJECT_NAME}_publisher standalone_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_publisher ${PROJECT_NAME}_nodes)
ament_target_dependencies(${PROJECT_NAME}_publisher
  rclcpp ${PROJECT_NAME}_nodes)

add_executable(${PROJECT_NAME}_subscriber standalone_subscriber.cpp)
target_link_libraries(${PROJECT_NAME}_subscriber ${PROJECT_NAME}_nodes)
ament_target_dependencies(${PROJECT_NAME}_subscriber
  rclcpp ${PROJECT_NAME}_nodes)

add_executable(${PROJECT_NAME}_run_time compose_at_run_time.cpp)
ament_target_dependencies(${PROJECT_NAME}_run_time rclcpp class_loader)

add_executable(${PROJECT_NAME}_compile_time compose_at_compile_time.cpp)
target_link_libraries(${PROJECT_NAME}_compile_time ${PROJECT_NAME}_nodes)
ament_target_dependencies(${PROJECT_NAME}_compile_time rclcpp class_loader)

install(TARGETS
  ${PROJECT_NAME}_nodes
  ${PROJECT_NAME}_publisher
  ${PROJECT_NAME}_subscriber
  ${PROJECT_NAME}_run_time
  ${PROJECT_NAME}_compile_time
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
