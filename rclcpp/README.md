# Minimal rclcpp wait-set cookbook recipes

This package contains a few different strategies for creating nodes which use `rclcpp::waitset` 
to wait and handle ROS entities, that is, subscribers, timers, clients, services, guard 
conditions and waitables.

* `wait_set_topics_and_timer.cpp`: Simple example using multiple subscriptions, 
  publishers, and a timer.
* `wait_set_random_order.cpp`: An example showing user-defined 
  data handling and a random publisher. `executor_random_order.cpp` run the same node logic 
  using `SingleThreadedExecutor` to compare the data handling order.  
* `wait_set_and_executor_composition.cpp`: An example showing how to combine a  
  `SingleThreadedExecutor` and a wait-set.
* `wait_set_topics_with_different_rate.cpp`: An example showing how to use a custom trigger 
  condition to handle topics with different topic rates.
