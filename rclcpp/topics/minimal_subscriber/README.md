# Minimal subscriber cookbook recipes

This package contains a few different strategies for creating nodes which receive messages:
 * `lambda.cpp` uses a C++11 lambda function
 * `member_function.cpp` uses a C++ member function callback
 * `not_composable.cpp` uses a global function callback without a Node subclass
 * `wait_set_subscriber.cpp` uses a `rclcpp::WaitSet` to wait and poll for data
 * `static_wait_set_subscriber.cpp` uses a `rclcpp::StaticWaitSet` to wait and poll for data
 * `time_triggered_wait_set_subscriber.cpp` uses a `rclcpp::Waitset` and a timer to poll for data
   periodically
 * `content_filtering.cpp` uses the content filtering feature for Subscriptions

Note that `not_composable.cpp` instantiates a `rclcpp::Node` _without_ subclassing it.
This was the typical usage model in ROS 1, but this style of coding is not compatible with composing multiple nodes into a single process.
Thus, it is no longer the recommended style for ROS 2.

All of these nodes do the same thing: they create a node called `minimal_subscriber` and subscribe to a topic named `topic` which is of datatype `std_msgs/String`.
When a message arrives on that topic, the node prints it to the screen.
We provide multiple examples of different coding styles which achieve this behavior in order to demonstrate that there are many ways to do this in ROS 2.

The following examples `wait_set_subscriber.cpp`, `static_wait_set_subscriber.cpp` and `time_triggered_wait_set_subscriber.cpp` show how to use a subscription in a node using a `rclcpp` wait-set.
This is not a common use case in ROS 2 so this is not the recommended strategy to  use by-default.
This strategy makes sense in some specific situations, for example when the developer needs to have more control over callback order execution, to create custom triggering conditions or to use the timeouts provided by the  wait-sets.   

The example `content_filtering.cpp` shows how to use the content filtering feature for Subscriptions.
