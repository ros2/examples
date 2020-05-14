# Minimal publisher examples

This package contains a few different strategies for creating short nodes which blast out messages.
The `talker_timer_lambda` and `talker_timer_member_function` recipes create subclasses of `rclcpp::Node` and set up an `rclcpp::timer` to periodically call functions which publish messages.
The `talker_without_subclass` recipe instead instantiates a `rclcpp::Node` object *without* subclassing it, which works, but is not compatible with composition, and thus is no longer the recommended style for ROS 2 coding.
