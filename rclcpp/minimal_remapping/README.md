# Minimal remapping cookbook recipes

This package contains strategies for remapping names:
 * `node_constructor.cpp` passes node-specific remap rules to the constructor `rclcpp::Node(...)`
 * `rclcpp_init.cpp` passes remap rules via the command line argument handling of `rclcpp::init()`

These nodes all do the same thing: they create a node called `minimal_remapping_listener` and subscribe to a topic named `/foo/bar` which is of datatype `std_msgs/String`.
When a message arrives on that topic, the node prints it to the screen.
The topic name is remapped from `/foo/bar` to `/bar/foo`.
The rule provided to `rclcpp::init` would affects all nodes in the process, while the rule provided to `rclcpp::Node(...)` would only affect that node.
