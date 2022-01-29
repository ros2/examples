cmake_minimum_required(VERSION 3.8)
project(random_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Action Interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Randomizer.action"
)
ament_export_dependencies(
	rosidl_default_runtime
)

# Action Client
add_library(action_client SHARED
  src/randomizer_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "RANDOM_GENERATOR_BUILDING_DLL")
ament_target_dependencies(action_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client 
  PLUGIN "random_generator::RandomizerActionClient" 
  EXECUTABLE randomizer_action_client)
install(TARGETS action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Action Server
add_library(action_server SHARED
  src/randomizer_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "RANDOM_GENERATOR_BUILDING_DLL")
ament_target_dependencies(action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server 
  PLUGIN "random_generator::RandomizerActionServer"
  EXECUTABLE randomizer_action_server)
install(TARGETS action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Include the action interface to the client and server, both the library and executable
rosidl_target_interfaces(action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(action_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(randomizer_action_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(randomizer_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Finally
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
