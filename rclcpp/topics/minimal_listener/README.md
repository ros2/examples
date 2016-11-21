# Minimal "listener" cookbook recipes

This package contains a few different strategies for creating short nodes
which receive messages:
 * `listener_timer_lambda` uses a C++11 lambda function for a callback
 * `listener_member_function` uses an "old school" C++ member function callback
 * `listener_without_subclass` uses a global function callback
 
Note that `listener_without_subclass` instantiates a `rclcpp::Node` _without_
subclassing it, which works, but is not compatible with composition, and thus
is no longer the recommended style for ROS 2 coding.
