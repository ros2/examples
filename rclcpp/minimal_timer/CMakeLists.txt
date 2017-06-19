cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_timer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME}_lambda lambda.cpp)
ament_target_dependencies(${PROJECT_NAME}_lambda rclcpp)

add_executable(${PROJECT_NAME}_member_function member_function.cpp)
ament_target_dependencies(${PROJECT_NAME}_member_function rclcpp)

install(TARGETS
  ${PROJECT_NAME}_lambda
  ${PROJECT_NAME}_member_function
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
