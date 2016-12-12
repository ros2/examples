# Threading cookbook recipes

This package contains examples for using rclpy in a thread.

The `data_publisher` recipe publishes data at a high frequency, and the `data_plotter` recipe plots
this data in the main thread while processing the subscription callback calls in a separate thread.
Slow plot updates do not block the reception of the frequent data.
