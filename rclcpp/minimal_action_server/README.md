# Minimal action server cookbook recipes

This package contains an example which shows how to create an action server.

"Composable" refers to the ability for a node to be used as part of a Composition (running multiple nodes in a single process).

Note that not_composable.cpp instantiates a rclcpp::Node without subclassing it. This was the typical usage model in ROS 1, but unfortunately this style of coding is not compatible with composing multiple nodes into a single process. Thus, it is no longer the recommended style for ROS 2.
