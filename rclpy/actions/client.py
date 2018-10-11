# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from example_interfaces.action import Fibonacci

import rclpy
from rclpy.node import Node


def feedback_cb(logger, feedback):
    logger.info('got feedback {0}'.format(repr(feedback))


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_action_client')

    action_client = self.create_action_client(Fibonacci, 'fibonacci')

    goal_msg = Fibonacci.Goal()
    goal_msg.order = 10

    future = self.action_client.send_goal_async(
        goal_msg, feedback_callback=lambda feedback: feedback_cb(node.get_logger(), feedback))

    while rclpy.ok() and not future.done():
        rclpy.spin_once(node)

    if future.done():
        if future.result() is not None:
            node.get_logger().info(
                'Action succeeded {0}'.format(repr(future.result())))
        else:
            node.get_logger().info(
                'Action request failed {0}'.format(repr(future.exception())))
        break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
