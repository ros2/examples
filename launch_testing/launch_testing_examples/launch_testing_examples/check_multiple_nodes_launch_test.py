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
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_actions = []
    node_names = []

    for i in range(3):
        launch_actions.append(
            launch_ros.actions.Node(
                executable='talker',
                package='demo_nodes_cpp',
                name='demo_node_' + str(i)
            )
        )
        node_names.append('demo_node_' + str(i))

    launch_actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(launch_actions), {'node_list': node_names}


class CheckMultipleNodesLaunched(unittest.TestCase):

    def test_nodes_successful(self, node_list):
        """Check if all the nodes were launched correctly."""
        # Method 1
        wait_for_nodes_1 = WaitForNodes(node_list, timeout=5.0)
        assert wait_for_nodes_1.wait()
        assert wait_for_nodes_1.get_nodes_not_found() == set()
        wait_for_nodes_1.shutdown()

        # Method 2
        with WaitForNodes(node_list, timeout=5.0) as wait_for_nodes_2:
            print('All nodes were found !')
            assert wait_for_nodes_2.get_nodes_not_found() == set()

    def test_node_does_not_exist(self, node_list):
        """Insert a invalid node name that should not exist."""
        invalid_node_list = node_list + ['invalid_node']

        # Method 1
        wait_for_nodes_1 = WaitForNodes(invalid_node_list, timeout=1.0)
        assert not wait_for_nodes_1.wait()
        assert wait_for_nodes_1.get_nodes_not_found() == {'invalid_node'}
        wait_for_nodes_1.shutdown()

        # Method 2
        with pytest.raises(RuntimeError):
            with WaitForNodes(invalid_node_list, timeout=1.0):
                pass


# TODO (adityapande-1995): Move WaitForNodes implementation to launch_testing_ros
# after https://github.com/ros2/rclpy/issues/831 is resolved
class WaitForNodes:
    """
    Wait to discover supplied nodes.

    Example usage:
    --------------
    # Method 1, calling wait() and shutdown() manually
    def method_1():
        node_list = ['foo', 'bar']
        wait_for_nodes = WaitForNodes(node_list, timeout=5.0)
        assert wait_for_nodes.wait()
        print('Nodes found!')
        assert wait_for_nodes.get_nodes_not_found() == set()
        wait_for_nodes.shutdown()

    # Method 2, using the 'with' keyword
    def method_2():
        with WaitForNodes(['foo', 'bar'], timeout=5.0) as wait_for_nodes:
            assert wait_for_nodes.get_nodes_not_found() == set()
            print('Nodes found!')
    """

    def __init__(self, node_names, timeout=5.0):
        self.node_names = node_names
        self.timeout = timeout
        self.__ros_context = rclpy.Context()
        rclpy.init(context=self.__ros_context)
        self._prepare_node()

        self.__expected_nodes_set = set(node_names)
        self.__nodes_found = None

    def _prepare_node(self):
        self.__node_name = '_test_node_' +\
            ''.join(random.choices(string.ascii_uppercase + string.digits, k=10))
        self.__ros_node = Node(node_name=self.__node_name, context=self.__ros_context)

    def wait(self):
        start = time.time()
        flag = False
        print('Waiting for nodes')
        while time.time() - start < self.timeout and not flag:
            flag = all(name in self.__ros_node.get_node_names() for name in self.node_names)
            time.sleep(0.3)

        self.__nodes_found = set(self.__ros_node.get_node_names())
        self.__nodes_found.remove(self.__node_name)
        return flag

    def shutdown(self):
        self.__ros_node.destroy_node()
        rclpy.shutdown(context=self.__ros_context)

    def __enter__(self):
        if not self.wait():
            raise RuntimeError('Did not find all nodes !')

        return self

    def __exit__(self, exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception('Exception occured, value: ', exep_value)
        self.shutdown()

    def get_nodes_found(self):
        return self.__nodes_found

    def get_nodes_not_found(self):
        return self.__expected_nodes_set - self.__nodes_found
