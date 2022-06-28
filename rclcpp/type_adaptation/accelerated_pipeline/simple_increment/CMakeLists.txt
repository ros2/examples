# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(simple_increment)

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
find_package(example_type_adapters REQUIRED)

find_package(CUDA 10.2 REQUIRED)

# Enable NVTX markers for improved profiling
add_definitions(-DUSE_NVTX)
link_directories("${CUDA_TOOLKIT_ROOT_DIR}/lib64")
link_libraries("nvToolsExt")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

# CUDA functions
cuda_add_library(cuda_functions SHARED
  src/cuda/cuda_functions.cu
)

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
  ${CUDA_LIBRARIES}
)

ament_target_dependencies(inc_node
  rclcpp
  rclcpp_components
  sensor_msgs
  example_type_adapters
)

rclcpp_components_register_node(inc_node
  PLUGIN "type_adaptation::simple_increment::IncNode"
  EXECUTABLE type_adapt_inc_node)


install(TARGETS
  cuda_functions
  inc_node
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

ament_auto_package()
