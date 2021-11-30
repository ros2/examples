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
import shutil
import tempfile
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest

import yaml


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    rosbag_dir = os.path.join(tempfile.mkdtemp(), 'test_bag')

    node_list = [
        launch_ros.actions.Node(
            executable='talker',
            package='demo_nodes_cpp',
            name='demo_node'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', rosbag_dir],
            output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ]

    return launch.LaunchDescription(node_list), {'rosbag_dir': rosbag_dir}


class DelayShutdown(unittest.TestCase):

    def test_delay(self):
        """Delay the shutdown of processes so that rosbag can record some messages."""
        time.sleep(3)


# TODO : Test fails on windows, to be fixed
# https://github.com/ros2/rosbag2/issues/926
if os.name != 'nt':
    @launch_testing.post_shutdown_test()
    class TestFixtureAfterShutdown(unittest.TestCase):

        rosbag_dir = None

        def test_rosbag_record(self, rosbag_dir):
            """Check if the rosbag2 recording was successful."""
            with open(os.path.join(rosbag_dir, 'metadata.yaml'), 'r') as file:
                metadata = yaml.safe_load(file)
                assert metadata['rosbag2_bagfile_information']['message_count'] > 0
                print('The following topics received messages:')
                for item in metadata['rosbag2_bagfile_information']['topics_with_message_count']:
                    print(item['topic_metadata']['name'], 'recieved ', item['message_count'],
                          ' messages')

            TestFixtureAfterShutdown.rosbag_dir = rosbag_dir

        @classmethod
        def tearDownClass(cls):
            """Delete the rosbag directory."""
            print('Deleting ', cls.rosbag_dir)
            shutil.rmtree(cls.rosbag_dir.replace('test_bag', ''))
