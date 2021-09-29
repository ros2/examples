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
import launch_testing.actions
import launch_testing.markers
import pytest


# This function specifies the processes to be run for our test
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch a simple process to print 'hello_world'."""
    return launch.LaunchDescription([
        # Launch a process to test
        launch.actions.ExecuteProcess(
            cmd=['echo', 'hello_world'],
            shell=True
        ),
        # Tell launch to start the test
        launch_testing.actions.ReadyToTest()
    ])


# This is our test fixture. Each method is a test case.
# These run alongside the processes specified in generate_test_description()
class TestHelloWorldProcess(unittest.TestCase):

    def test_read_stdout(self, proc_output):
        """Check if 'hello_world' was found in the stdout."""
        # 'proc_output' is an object added automatically by the launch_testing framework.
        # It captures the outputs of the processes launched in generate_test_description()
        # Refer to the documentation for further details.
        proc_output.assertWaitFor('hello_world', timeout=10, stream='stdout')


# These tests are run after the processes in generate_test_description() have shutdown.
@launch_testing.post_shutdown_test()
class TestHelloWorldShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
