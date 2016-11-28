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

import sys

import rclpy

from std_msgs.msg import String


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args)

    node = rclpy.create_node('minimal_publisher')
    publisher_ = node.create_publisher(String, 'topic')

    msg = String()
    i = 0

    def timer_callback():
        nonlocal i
        msg.data = 'Hello World: %d' % i
        i += 1
        print('Publishing: "%s"' % msg.data)
        publisher_.publish(msg)

    timer = node.create_timer(0.5, timer_callback)

    while rclpy.ok():
        rclpy.spin_once(node)

    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
