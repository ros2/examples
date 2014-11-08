# cmake_minimum_required(VERSION 2.8.3)

# project(userland)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

# find_package(ros_middleware_interface REQUIRED)

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(ros_middleware_implementation REQUIRED)
# find_package(simple_msgs REQUIRED)

# ament_package()

# include_directories(include)

# # Get the implementations ahead of time
# get_available_ros_middleware_implementations(middleware_implementations)
# foreach(middleware_impl ${middleware_implementations})
#   find_package("${middleware_impl}" REQUIRED)
# endforeach()

# function(build_executable executable)
#   # Build the normal executable
#   add_executable(${executable} ${ARGN})
#   ament_target_dependencies(${executable}
#     "rclcpp"
#     "ros_middleware_implementation"
#     "simple_msgs")

#   install(TARGETS ${executable} DESTINATION bin)

#   # Build an executable for each ROS middleware implementation
#   foreach(middleware_impl_tmp ${middleware_implementations})
#     add_executable(${executable}__${middleware_impl_tmp} ${ARGN})
#     ament_target_dependencies(${executable}__${middleware_impl_tmp}
#       "rclcpp"
#       "${middleware_impl_tmp}"
#       "simple_msgs")

#     install(TARGETS ${executable}__${middleware_impl_tmp} DESTINATION bin)
#   endforeach()

# endfunction()

# # Build library in the component style
# # rclcpp_create_node(node_name
# #   SOURCES
# #     src/component/component_talker.cpp
# #   DEPENDENCIES
# #     "rclcpp"
# #     "ros_middleware_implementation"
# #     "simple_msgs"
# #   # INCLUDE_DIRS
# #   #   ${Boost_INCLUDE_DIRS}
# #   # LINK_LIBRARIES
# #   #   ${Boost_LIBRARIES}
# # )
# # # libnode_name.so
# # # entry in the node plugin list
# # # node_name_node executable
# add_library(component_talker SHARED src/component/component_talker.cpp)
# ament_target_dependencies(component_talker
#   "rclcpp"
#   "ros_middleware_implementation"
#   "simple_msgs")
# install(TARGETS component_talker DESTINATION lib)
# # Create an executable for this component style node
# rclcpp_create_node_main(component_talker)

# # Build pub/sub command line programs
# build_executable(publisher src/publisher.cpp)

# build_executable(subscriber src/subscriber.cpp)

# build_executable(prototype_intraprocess src/prototype_intraprocesses.cpp)

# # Build ros1 like examples
# build_executable(talker src/ros1_like/talker.cpp)

# build_executable(listener src/ros1_like/listener.cpp)

# # Build more sofisticated examples
# build_executable(different_groups src/explicit/different_groups.cpp)
# build_executable(two_nodes src/explicit/two_nodes.cpp)
# # build_executable(two_nodes_no_intra src/explicit/two_nodes_no_intra.cpp)
