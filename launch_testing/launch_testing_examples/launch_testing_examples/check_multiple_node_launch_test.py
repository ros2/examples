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

import os
import sys
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

import yaml


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    path_to_test = os.path.dirname(__file__)

    # Default number of nodes to be launched
    n = 5
    for item in sys.argv[::-1]:
        if 'number_of_nodes:=' in item:
            n = int(item.strip('number_of_nodes:='))

    print('Launching ', n, 'nodes !')

    node_list = []
    node_names = []
    for i in range(n):
        node_list.append(
            launch_ros.actions.Node(
                executable=sys.executable,
                arguments=[os.path.join(path_to_test, 'talker.py')],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='demo_node_' + str(i),
            )
        )
        node_names.append('demo_node_' + str(i))

    # Record all the data in a rosbag
    node_list.append(
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', 'test_bag'],
            output='screen'
        )
    )

    node_list.append(launch_testing.actions.ReadyToTest())

    return launch.LaunchDescription(node_list), {'node_list': node_names}


class TestFixture(unittest.TestCase):

    def test_node_start(self, proc_output, node_list):
        """Check if all the nodes were launched correctly."""
        rclpy.init()
        node = DummyNode('test_node')
        time.sleep(3)
        assert node.wait_for_nodes(node_list, 8.0), 'Nodes not found !'
        rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestFixtureAfterShutdown(unittest.TestCase):

    def test_rosbag_record(self):
        """Check if the rosbag2 recording was successful."""
        with open('test_bag/metadata.yaml', 'r') as stream:
            metadata = yaml.safe_load(stream)
            assert metadata['rosbag2_bagfile_information']['message_count'] > 0


class DummyNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)

    def wait_for_nodes(self, node_names, timeout=8.0):
        start = time.time()
        flag = False
        print('Waiting for nodes...')
        while time.time() - start < timeout and not flag:
            flag = all(name in self.get_node_names() for name in node_names)
            time.sleep(0.1)

        return flag
