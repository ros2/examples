cmake_minimum_required(VERSION 3.5)

project(type_adapt_example)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -funwind-tables)  
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(CUDA 10.2 REQUIRED)

set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};--expt-relaxed-constexpr")
include_directories(include)
# Node CUDA functions
cuda_add_library(cuda_functions SHARED
  src/cuda_functions.cu
  src/juliaset.cu
)

# Type Adapter
add_library(nvidia_cuda_type_adapter SHARED
  src/nvidia_cuda_sensor_msgs_image_type_adapter.cpp
)
target_include_directories(nvidia_cuda_type_adapter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(nvidia_cuda_type_adapter
  cuda_functions
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(nvidia_cuda_type_adapter
  rclcpp
  sensor_msgs
)

# Node1
add_library(node1 SHARED
  src/node1.cpp
)
target_include_directories(node1 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(node1
  cuda_functions
  nvidia_cuda_type_adapter
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(node1
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(node1
  PLUGIN "type_adapt_example::Node1"
  EXECUTABLE type_adapt_node1)

# Node2
add_library(node2 SHARED
  src/node2.cpp
)
target_include_directories(node2 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(node2
  cuda_functions
  nvidia_cuda_type_adapter
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(node2
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(node2
  PLUGIN "type_adapt_example::Node2"
  EXECUTABLE type_adapt_node2)

# Node3
add_library(node3 SHARED
  src/node3.cpp
)
target_include_directories(node3 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(node3
  cuda_functions
  nvidia_cuda_type_adapter
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(node3
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(node3
  PLUGIN "type_adapt_example::Node3"
  EXECUTABLE type_adapt_node3)

# IncNode
add_library(inc_node SHARED
  src/inc_node.cpp
)
target_include_directories(inc_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(inc_node
  cuda_functions
  nvidia_cuda_type_adapter
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(inc_node
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(inc_node
  PLUGIN "type_adapt_example::IncNode"
  EXECUTABLE type_adapt_inc_node)

# JuliasetNode
add_library(juliaset_node SHARED
  src/juliaset_node.cpp
)
target_include_directories(juliaset_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(juliaset_node
  cuda_functions
  nvidia_cuda_type_adapter
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(juliaset_node
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(juliaset_node
  PLUGIN "type_adapt_example::JuliasetNode"
  EXECUTABLE type_adapt_juliaset_node)

# ColorizeNode
add_library(colorize_node SHARED
  src/colorize_node.cpp
)
target_include_directories(colorize_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(colorize_node
  cuda_functions
  nvidia_cuda_type_adapter
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(colorize_node
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(colorize_node
  PLUGIN "type_adapt_example::ColorizeNode"
  EXECUTABLE type_adapt_colorize_node)

# MapNode
add_library(map_node SHARED
  src/map_node.cpp
)
target_include_directories(map_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(map_node
  cuda_functions
  nvidia_cuda_type_adapter
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(map_node
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(map_node
  PLUGIN "type_adapt_example::MapNode"
  EXECUTABLE type_adapt_map_node)

install(TARGETS
  cuda_functions
  nvidia_cuda_type_adapter
  node1
  node2
  node3
  inc_node
  juliaset_node
  map_node
  colorize_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
