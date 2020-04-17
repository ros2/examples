# Minimal timer examples

This package contains a few different strategies for creating short nodes which have timers.
The `timer_lambda` and `timer_member_function` examples create subclasses of `rclcpp::Node` and set up an `rclcpp::timer` to periodically call functions which just print Hello to the console. They do the same thing, just using different C++ language features.
