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

import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
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
        with WaitForTopics([('chatter', String)], timeout=15.0):
            print('Topic received messages !')
