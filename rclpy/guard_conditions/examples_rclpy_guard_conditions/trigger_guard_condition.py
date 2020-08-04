# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import threading
import time

import rclpy
from rclpy.executors import GuardCondition


def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('DemoGuardCondition')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    def guard_condition_callback():
        node.get_logger().info('called the guard conditions callback')

    def delay_triggering_guard_condition(guard_condition: GuardCondition, delay_in_seconds: int):
        time.sleep(delay_in_seconds)
        node.get_logger().info('triggering guard condition')
        guard_condition.trigger()

    guard_condition = node.create_guard_condition(guard_condition_callback)

    delay_in_seconds: float = 5.0
    while True:
        # start a thread that will trigger the guard condition after a delay
        thread = threading.Thread(
            target=delay_triggering_guard_condition,
            args=(guard_condition, delay_in_seconds))
        thread.start()

        # `spin_once` will block until the guard condition is triggered
        node.get_logger().info("waiting for 'spin_once' to finish...")
        executor.spin_once()
        node.get_logger().info("...'spin_once' finished!\n")


if __name__ == '__main__':
    main()
