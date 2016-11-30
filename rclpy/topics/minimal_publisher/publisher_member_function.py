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

from std_msgs.msg import String


class MinimalPublisher:

    def __init__(self, node):
        self.publisher_ = node.create_publisher(String, 'topic')
        timer_period = 0.5  # seconds
        self.timer = node.create_timer(timer_period, self.timer_callback)
        self.msg = String()
        self.i = 0

    def timer_callback(self):
        self.msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(self.msg)
        print('Publishing: "%s"' % self.msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args)

    node = rclpy.create_node('minimal_publisher')

    minimal_publisher = MinimalPublisher(node)
    minimal_publisher  # prevent unused variable warning

    while rclpy.ok():
        rclpy.spin_once(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
