# Copyright 2025 Sony Group Corporation.
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
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher_with_wait_for_all_acked')
        reliable_qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE)
        self.publisher_ = self.create_publisher(String, 'topic', reliable_qos)

        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.max_count = 10

    def wait_for_all_acked(self):
        self.get_logger().info('Waiting for all messages to be acknowledged...')
        acknowledged = self.publisher_.wait_for_all_acked(Duration(seconds=3))
        if acknowledged:
            self.get_logger().info('All messages acknowledged.')
        else:
            self.get_logger().warn('Not all messages were acknowledged.')

    def timer_callback(self):
        if self.i < self.max_count:
            msg = String()
            msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1
        else:
            self.timer.cancel()
            self.wait_for_all_acked()
            self.i = 0
            self.timer.reset()


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    try:
        rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
