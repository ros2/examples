# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import random
import string
from threading import Event
from threading import Thread
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            executable='talker',
            package='demo_nodes_cpp',
            name='demo_node_1'
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestFixture(unittest.TestCase):

    def test_check_if_msgs_published(self):
        with WaitForTopics([('chatter', String)], timeout=5.0):
            print('Topic received messages !')


# TODO (adityapande-1995): Move WaitForTopics implementation to launch_testing_ros
# after https://github.com/ros2/rclpy/issues/831 is resolved
class WaitForTopics:
    """
    Wait to receive messages on supplied topics.

    Example usage:
    --------------
    from std_msgs.msg import String
    # Method 1, using the 'with' keyword
    def method_1():
        topic_list = [('topic_1', String), ('topic_2', String)]
        with WaitForTopics(topic_list, timeout=5.0):
            # 'topic_1' and 'topic_2' received at least one message each
            print('Given topics are receiving messages !')
    # Method 2, calling wait() and shutdown() manually
    def method_2():
        topic_list = [('topic_1', String), ('topic_2', String)]
        wait_for_topics = WaitForTopics(topic_list, timeout=5.0)
        assert wait_for_topics.wait()
        print('Given topics are receiving messages !')
        print(wait_for_topics.topics_not_received()) # Should be an empty set
        print(wait_for_topics.topics_received()) # Should be {'topic_1', 'topic_2'}
        wait_for_topics.shutdown()
    """

    def __init__(self, topic_tuples, timeout=5.0):
        self.topic_tuples = topic_tuples
        self.timeout = timeout
        self.__ros_context = rclpy.Context()
        rclpy.init(context=self.__ros_context)
        self.__ros_executor = SingleThreadedExecutor(context=self.__ros_context)

        self._prepare_ros_node()

        # Start spinning
        self.__running = True
        self.__ros_spin_thread = Thread(target=self._spin_function)
        self.__ros_spin_thread.start()

    def _prepare_ros_node(self):
        node_name = '_test_node_' +\
            ''.join(random.choices(string.ascii_uppercase + string.digits, k=10))
        self.__ros_node = _WaitForTopicsNode(name=node_name, node_context=self.__ros_context)
        self.__ros_executor.add_node(self.__ros_node)

    def _spin_function(self):
        while self.__running:
            self.__ros_executor.spin_once(1.0)

    def wait(self):
        self.__ros_node.start_subscribers(self.topic_tuples)
        return self.__ros_node.msg_event_object.wait(self.timeout)

    def shutdown(self):
        self.__running = False
        self.__ros_spin_thread.join()
        self.__ros_node.destroy_node()
        rclpy.shutdown(context=self.__ros_context)

    def topics_received(self):
        """Topics that received at least one message."""
        return self.__ros_node.received_topics

    def topics_not_received(self):
        """Topics that did not receive any messages."""
        return self.__ros_node.expected_topics - self.__ros_node.received_topics

    def __enter__(self):
        if not self.wait():
            raise RuntimeError('Did not receive messages on these topics: ',
                               self.topics_not_received())
        return self

    def __exit__(self, exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception('Exception occured, value: ', exep_value)
        self.shutdown()


class _WaitForTopicsNode(Node):
    """Internal node used for subscribing to a set of topics."""

    def __init__(self, name='test_node', node_context=None):
        super().__init__(node_name=name, context=node_context)
        self.msg_event_object = Event()

    def start_subscribers(self, topic_tuples):
        self.subscriber_list = []
        self.expected_topics = {name for name, _ in topic_tuples}
        self.received_topics = set()

        for topic_name, topic_type in topic_tuples:
            # Create a subscriber
            self.subscriber_list.append(
                self.create_subscription(
                    topic_type,
                    topic_name,
                    self.callback_template(topic_name),
                    10
                )
            )

    def callback_template(self, topic_name):

        def topic_callback(data):
            if topic_name not in self.received_topics:
                self.get_logger().debug('Message received for ' + topic_name)
                self.received_topics.add(topic_name)
            if self.received_topics == self.expected_topics:
                self.msg_event_object.set()

        return topic_callback
