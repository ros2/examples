# Launch testing examples

This package contains simple use cases for the ``launch`` and ``launch_testing`` packages.
These are designed to help beginners get started with these packages and help them understand the concepts.

## Examples

### `check_node_launch_test.py`

Usage:

```sh
launch_test launch_testing_examples/check_node_launch_test.py
```

There might be situations where nodes, once launched, take some time to actually start and we need to wait for the node to start to perform some action.
We can simulate this using ``launch.actions.TimerAction``.
This example shows one way to detect when a node has been launched.
We delay the launch by 5 seconds, and wait for the node to start with a timeout of 20 seconds.

### `check_multiple_nodes_launch_test.py`

```sh
launch_test test/examples/check_multiple_nodes_launch_test.py
```

This test launches multiple nodes, and checks if they were launched successfully using the `WaitForNodes` utility.

### `record_rosbag_launch_test.py`

```sh
launch_test test/examples/record_rosbag_launch_test.py
```

This test launches a `talker` node, records the topics to a `rosbag` and makes sure that the messages were recorded successfully,
then deletes the bag file.

### `check_msgs_launch_test.py`

Usage:

```sh
launch_test launch_testing_examples/check_msgs_launch_test.py
```

Consider a problem statement where you need to launch a node and check if messages are published on a particular topic.
This example demonstrates how to do that, using a talker node.
It uses the ``Event`` object to end the test as soon as the first message is received on the chatter topic, with a timeout of 5 seconds.

### `set_param_launch_test.py`

Usage:

```sh
launch_test launch_testing_examples/set_param_launch_test.py
```

This example demonstrates how to launch a node, set a parameter in it and check if that was successful.

### `hello_world_launch_test.py`

Usage:

```sh
launch_test launch_testing_examples/hello_world_launch_test.py
```

This test is a simple example on how to use the ``launch_testing``.

It launches a process and asserts that it prints "hello_world" to ``stdout`` using ``proc_output.assertWaitFor()``.
Finally, it checks if the process exits normally (zero exit code).

The ``@launch_testing.markers.keep_alive`` decorator ensures that the launch process stays alive long enough for the tests to run.
