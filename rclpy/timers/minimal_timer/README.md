# Minimal Timer Example

This example demonstrates how to create a simple timer node in ROS 2 using rclpy.

## Overview

ROS 2 timers allow nodes to execute a callback function periodically.
They are commonly used for tasks such as monitoring loops, periodic updates,
or scheduled processing.

This example creates a node that executes a callback every second using
`create_timer()`.

## Running the Example

Run the node with:

```bash
python3 minimal_timer.py

```

You should see log messages printed once per second.

## Example output

Timer callback triggered
Timer callback triggered
Timer callback triggered

## Key Concept

`create_timer()` schedules a callback to be executed periodically.
