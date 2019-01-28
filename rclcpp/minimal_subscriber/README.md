# Minimal subscriber cookbook recipes

This package contains a few different strategies for creating nodes which receive messages:
 * `lambda.cpp` uses a C++11 lambda function
 * `member_function.cpp` uses a C++ member function callback
 * `not_composable.cpp` uses a global function callback without a Node subclass
 
Note that `not_composable.cpp` instantiates a `rclcpp::Node` _without_ subclassing it.
This was the typical usage model in ROS 1, but this style of coding is not compatible with composing multiple nodes into a single process.
Thus, it is no longer the recommended style for ROS 2.

All of these nodes do the same thing: they create a node called `minimal_listener` and subscribe to a topic named `topic` which is of datatype `std_msgs/String`.
When a message arrives on that topic, the node prints it to the screen.
We provide multiple examples of different coding styles which achieve this behavior in order to demonstrate that there are many ways to do this in ROS 2.
