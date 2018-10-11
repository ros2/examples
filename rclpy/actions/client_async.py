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


class MinimalActionClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_action_client')
        self.action_client = self.create_action_client(Fibonacci, 'fibonacci')

        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    async def timer_callback(self):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10

        future = self.action_client.send_goal_async(goal_msg)
        await future

        if future.exception() is not None:
            self.get_logger().error(
                'Action request failed {0}'.format(repr(future.exception())))
        else:
            self.get_logger().info(
                'Action succeeded {0}'.format(repr(future.result())))


def main(args=None):
    rclpy.init(args=args)

    node = MinimalActionClientAsync()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

