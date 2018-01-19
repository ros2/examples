# Minimal remapping cookbook recipes

This package contains strategies for remapping names:
 * `create_node.py` passes node-specific remap rules to `rclpy.create_node(...)`
 * `node.py` passes node-specific remap rules to `Node.__init__(...)`
 * `init.py` passes remap rules via the command line argument handling of `rclpy.init(...)`

These nodes all do the same thing: they create a node called `minimal_remapping_listener` and subscribe to a topic named `/foo/bar` which is of datatype `std_msgs/String`.
When a message arrives on that topic, the node prints it to the screen.
The topic name is remapped from `/foo/bar` to `/bar/foo`.
The rule provided to `rclpy.init(...)` would affects all nodes in the process, while the rule provided to `rclpy.create_node(...)` or `Node.__init__(...)` affects only that node.
