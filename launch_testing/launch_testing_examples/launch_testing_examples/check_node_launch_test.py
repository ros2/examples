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


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    path_to_test = os.path.dirname(__file__)

    return launch.LaunchDescription([
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
                    executable=sys.executable,
                    arguments=[os.path.join(path_to_test, 'talker.py')],
                    additional_env={'PYTHONUNBUFFERED': '1'},
                    name='demo_node_1',
                )
            ]),
        launch_testing.actions.ReadyToTest()
    ])


class TestFixture(unittest.TestCase):

    def test_node_start(self, proc_output):
        rclpy.init()
        node = MakeTestNode('test_node')
        assert node.wait_for_node('demo_node_1', 8.0), 'Node not found !'
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)

    def wait_for_node(self, node_name, timeout=8.0):
        start = time.time()
        flag = False
        print('Waiting for node...')
        while time.time() - start < timeout and not flag:
            flag = node_name in self.get_node_names()
            time.sleep(0.1)

        return flag
