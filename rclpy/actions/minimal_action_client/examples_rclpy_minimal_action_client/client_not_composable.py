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

from action_msgs.msg import GoalStatus

from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient


def feedback_cb(logger, feedback):
    logger.info('Received feedback: {0}'.format(feedback.feedback.sequence))


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_action_client')

    action_client = ActionClient(node, Fibonacci, 'fibonacci')

    node.get_logger().info('Waiting for action server...')

    action_client.wait_for_server()

    goal_msg = Fibonacci.Goal()
    goal_msg.order = 10

    node.get_logger().info('Sending goal request...')

    send_goal_future = action_client.send_goal_async(
        goal_msg, feedback_callback=lambda feedback: feedback_cb(node.get_logger(), feedback))

    rclpy.spin_until_future_complete(node, send_goal_future)

    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        node.get_logger().info('Goal rejected :(')
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info('Goal accepted :)')

    get_result_future = goal_handle.get_result_async()

    rclpy.spin_until_future_complete(node, get_result_future)

    result = get_result_future.result().result
    status = get_result_future.result().status
    if status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info(
           'Goal succeeded! Result: {0}'.format(result.sequence))
    else:
        node.get_logger().info('Goal failed with status code: {0}'.format(status))

    action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
