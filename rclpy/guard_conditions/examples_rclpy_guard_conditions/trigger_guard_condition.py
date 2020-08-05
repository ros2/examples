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

import rclpy


def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('demo_guard_condition')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    def guard_condition_callback():
        rclpy.shutdown()
        node.get_logger().info('guard callback called shutdown')

    def timer_callback():
        guard_condition.trigger()
        node.get_logger().info('timer callback triggered guard condition')

    node.create_timer(timer_period_sec=2, callback=timer_callback)
    guard_condition = node.create_guard_condition(guard_condition_callback)

    while rclpy.ok():
        # First loop: `spin_once` waits for timer to be ready, then calls
        #   the timer's callback, which triggers the guard condition.
        # Second loop: The guard condition is ready so it's callback is
        #   called. The callback calls shutdown, so the loop doesn't run
        #   again and the program exits.
        node.get_logger().info("waiting for 'spin_once' to finish...")
        executor.spin_once()
        node.get_logger().info("...'spin_once' finished!\n")


if __name__ == '__main__':
    main()
