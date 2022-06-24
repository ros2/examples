cmake_minimum_required(VERSION 3.5)

project(example_type_adapters)

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

# Enable NVTX markers for improved profiling
add_definitions(-DUSE_NVTX)
link_directories("${CUDA_TOOLKIT_ROOT_DIR}/lib64")
link_libraries("nvToolsExt")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Type Adapter
ament_auto_add_library(example_type_adapters SHARED
  src/image_container.cpp
)

target_include_directories(example_type_adapters PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(example_type_adapters
  ${CUDA_nvToolsExt_LIBRARY}
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(example_type_adapters
  rclcpp
  sensor_msgs
)

# Add headers
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories("include")

install(TARGETS
  example_type_adapters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
