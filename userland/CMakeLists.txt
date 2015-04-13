cmake_minimum_required(VERSION 2.8.3)

project(userland)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
endif()

find_package(rmw REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(simple_msgs REQUIRED)
find_package(userland_msgs REQUIRED)

ament_package()

include_directories(include)

# Get the implementations ahead of time
get_available_rmw_implementations(middleware_implementations)
foreach(middleware_impl ${middleware_implementations})
  find_package("${middleware_impl}" REQUIRED)
endforeach()

function(build_executable executable)
  # Build the normal executable
  add_executable(${executable} ${ARGN})
  ament_target_dependencies(${executable}
    "rcl_interfaces"
    "rclcpp"
    "rmw_implementation"
    "simple_msgs"
    "userland_msgs")

  install(TARGETS ${executable} DESTINATION bin)

  # Build an executable for each ROS middleware implementation
  foreach(middleware_impl_tmp ${middleware_implementations})
    add_executable(${executable}__${middleware_impl_tmp} ${ARGN})
    ament_target_dependencies(${executable}__${middleware_impl_tmp}
      "rcl_interfaces"
      "rclcpp"
      "${middleware_impl_tmp}"
      "simple_msgs"
      "userland_msgs")

    install(TARGETS ${executable}__${middleware_impl_tmp} DESTINATION bin)
  endforeach()

endfunction()

# Build library in the component style
# rclcpp_create_node(node_name
#   SOURCES
#     src/component/component_talker.cpp
#   DEPENDENCIES
#     "rclcpp"
#     "rmw_implementation"
#     "simple_msgs"
#   # INCLUDE_DIRS
#   #   ${Boost_INCLUDE_DIRS}
#   # LINK_LIBRARIES
#   #   ${Boost_LIBRARIES}
# )
# # libnode_name.so
# # entry in the node plugin list
# # node_name_node executable
add_library(component_talker SHARED src/component/component_talker.cpp)
ament_target_dependencies(component_talker
  "rcl_interfaces"
  "rclcpp"
  "rmw_implementation"
  "simple_msgs")
install(TARGETS component_talker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# Create an executable for this component style node
rclcpp_create_node_main(component_talker)

# Build pub/sub command line programs
build_executable(publisher src/publisher.cpp)

build_executable(subscriber src/subscriber.cpp)

build_executable(prototype_intraprocess src/prototype_intraprocesses.cpp)

# Build imu_double command line programs
build_executable(imu_publisher_double src/imu/imu_double/publisher.cpp)

build_executable(imu_subscriber_double src/imu/imu_double/subscriber.cpp)

# Build imu_int command line programs
build_executable(imu_publisher_int src/imu/imu_int/publisher.cpp)

build_executable(imu_subscriber_int src/imu/imu_int/subscriber.cpp)

# Build imu_float command line programs
build_executable(imu_publisher_float src/imu/imu_float/publisher.cpp)

build_executable(imu_subscriber_float src/imu/imu_float/subscriber.cpp)

# Build imu32 command line programs
build_executable(imu32_publisher src/imu/imu32/publisher.cpp)

build_executable(imu32_subscriber src/imu/imu32/subscriber.cpp)

# Build rosimu command line programs
build_executable(rosimu_publisher src/imu/rosimu/publisher.cpp)

build_executable(rosimu_subscriber src/imu/rosimu/subscriber.cpp)

# Build ros1 like examples
build_executable(talker src/ros1_like/talker.cpp)

build_executable(listener src/ros1_like/listener.cpp)

# Build more sofisticated examples
build_executable(different_groups src/explicit/different_groups.cpp)
build_executable(two_nodes src/explicit/two_nodes.cpp)
# build_executable(two_nodes_no_intra src/explicit/two_nodes_no_intra.cpp)

# Build services examples
build_executable(add_two_ints_client src/add_two_ints_client.cpp)
build_executable(add_two_ints_server src/add_two_ints_server.cpp)

# Build parameters examples
build_executable(parameters src/parameters.cpp)


if(AMENT_ENABLE_TESTING)
  add_subdirectory(test)
endif()
