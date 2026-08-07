# Async Receive Callback Client

## Summary

This package contains an example that includes a node that 
is a client to the *add_two_ints* service. It shows how
to receive the response in a callback, thus avoiding any
waiting mechanism that might impact in the spinning of 
the node. 

Please, note that the former example in package: 
*examples_rclcpp_async_client* has a very specific way for 
spinning that are avoid in this example. 

The goal of this package is helping programmers to understand
how asyncronous services work. 

## Usage

Launch:

```bash
ros2 run examples_rclcpp_delayed_service service_main
```

```bash
ros2 run examples_rclcpp_async_recv_cb_client client_main
```

And trigger the call to the service by:

```bash
ros2 topic pub --once /input_topic std_msgs/msg/Int32 "data: 5"
```

Try to issue more than one call by publishing inmediately another
value to /input_topic. 

You might listen to the topic that publishs the result:

```bash
ros2 topic echo /output_topic 
```

## TODO

* Resolve the following questions: 

> 1. What is the callback group for the response callback? 
> 2. Is it possible to set another callback group?

Actually, answer to 1 should be the default node callback group.
Answer to 2 should be at api documentation (or sources?). 