# Minimal Publisher Examples 

This package provides several minimal examples demonstrating how to implement
publisher nodes in ROS 2 using `rclcpp`.

These examples are designed as learning references for new ROS 2 developers
and illustrate different design patterns for publishing messages.

### member_function.cpp
Subclasses `rclcpp::Node` and uses a member function as the timer callback.

This pattern is commonly used in ROS 2 and supports node composition when nodes are structured as classes.

### lambda.cpp
Uses a C++ lambda function as the timer callback.

This approach is concise and suitable for simple publisher implementations.

Both the lambda and member function patterns follow common ROS 2 best practices.  
The only discouraged approach shown here is the `not_composable.cpp` example.

### not_composable.cpp
Creates a publisher without subclassing `rclcpp::Node`.

While this pattern works, it does not support node composition and is generally discouraged in modern ROS 2 applications.

### member_function_with_wait_for_all_acked.cpp
Demonstrates how to wait until published messages are acknowledged before proceeding.
Useful when reliable communication is required.

### member_function_with_unique_network_flow_endpoints.cpp
Shows advanced publisher configuration for unique network flow endpoints.
Useful in complex distributed networking scenarios.

## Build Instructions

From the root of your ROS 2 workspace:

```bash
colcon build --packages-select examples_rclcpp_minimal_publisher
source install/setup.bash
```
Each example corresponds to a different executable built from the source files listed above. The executable name matches the corresponding `
.cpp` filename

## Run Example

```bash
ros2 run examples_rclcpp_minimal_publisher member_function
```
