# Copyright 2016 Open Source Robotics Foundation, Inc.
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
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import String


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = rclpy.create_node('minimal_publisher')
            publisher = node.create_publisher(String, 'topic', 10)

            msg = String()
            i = 0

            def timer_callback():
                nonlocal i
                msg.data = 'Hello World: %d' % i
                i += 1
                node.get_logger().info('Publishing: "%s"' % msg.data)
                publisher.publish(msg)

            timer_period = 0.5  # seconds
            timer = node.create_timer(timer_period, timer_callback)
            timer  # Quiet flake8 warnings about unused variable

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
