# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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

project(julia_set)

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

set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};--expt-relaxed-constexpr")
include_directories(include)

# Julia Set CUDA
cuda_add_library(julia_set_cuda SHARED
  src/cuda/julia_set.cu
)

# Julia Set Node
add_library(julia_set_node SHARED
  src/julia_set_node.cpp
)

target_include_directories(julia_set_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)

target_link_libraries(julia_set_node
  julia_set_cuda
  ${CUDA_LIBRARIES}
  ${CUDA_nvToolsExt_LIBRARY}
)

ament_target_dependencies(julia_set_node
  rclcpp
  rclcpp_components
  sensor_msgs
  example_type_adapters
)

rclcpp_components_register_node(julia_set_node
  PLUGIN "type_adaptation::julia_set::JuliaSetNode"
  EXECUTABLE type_adapt_julia_set_node)

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
  julia_set_cuda
  ${CUDA_LIBRARIES}
  ${CUDA_nvToolsExt_LIBRARY}
)

ament_target_dependencies(colorize_node
  rclcpp
  rclcpp_components
  sensor_msgs
  example_type_adapters
)

rclcpp_components_register_node(colorize_node
  PLUGIN "type_adaptation::julia_set::ColorizeNode"
  EXECUTABLE type_adapt_colorize_node)

# MapNode
ament_auto_add_library(map_node SHARED
  src/map_node.cpp
)

target_include_directories(map_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)

target_link_libraries(map_node
  julia_set_cuda
  ${CUDA_LIBRARIES}
  ${CUDA_nvToolsExt_LIBRARY}
)

ament_target_dependencies(map_node
  rclcpp
  rclcpp_components
  sensor_msgs
  example_type_adapters
)

rclcpp_components_register_node(map_node
  PLUGIN "type_adaptation::julia_set::MapNode"
  EXECUTABLE type_adapt_map_node)


install(TARGETS
  julia_set_cuda
  julia_set_node
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

ament_auto_package()
