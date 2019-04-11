# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import time

from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


logger = None


def cancel_callback(goal_handle):
    logger.info('Received cancel request')
    return CancelResponse.ACCEPT


async def execute_callback(goal_handle):
    """Executes the goal."""
    logger.info('Executing goal...')

    # Append the seeds for the fibonacci sequence
    feedback_msg = Fibonacci.Feedback()
    feedback_msg.sequence = [0, 1]

    # Start executing the action
    for i in range(1, goal_handle.request.order):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            logger.info('Goal canceled')
            return Fibonacci.Result()

        # Update Fibonacci sequence
        feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

        logger.info('Publishing feedback: {0}'.format(feedback_msg.sequence))

        # Publish feedback
        goal_handle.publish_feedback(feedback_msg)

        # Sleep for demonstration purposes
        time.sleep(1)

    goal_handle.succeed()

    # Populate result message
    result = Fibonacci.Result()
    result.sequence = feedback_msg.sequence

    logger.info('Returning result: {0}'.format(result.sequence))

    return result


def main(args=None):
    global logger
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_action_server')
    logger = node.get_logger()

    # Use a ReentrantCallbackGroup to enable processing multiple goals concurrently
    # Default goal callback accepts all goals
    # Default cancel callback rejects cancel requests
    action_server = ActionServer(
        node,
        Fibonacci,
        'fibonacci',
        execute_callback=execute_callback,
        cancel_callback=cancel_callback,
        callback_group=ReentrantCallbackGroup())

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(node, executor=executor)

    action_server.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
