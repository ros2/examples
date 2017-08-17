# Copyright 2017 Open Source Robotics Foundation, Inc.
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


class Listener(rclpy.Node):
    def __init__(self):
        super().__init__('listener')
        # rclcpp adds subscriptions to a mutually exclusive callback group be default
        # rclpy should add them to a reentrant group to match the behavior of rospy
        self.sub = self.create_subscription(String, 'chatter', self.chatter_callback)

    def chatter_callback(self, msg):
        print('I heard: [%s]' % msg.data)


def main(args=None):
    # Run standalone
    rclpy.init(args=args)
    try:
        # rclcpp::spin() creates a SingleThreadedExecutor.
        # rclpy should create a MultiThreadedExecutor instead because rospy
        # executes every callback in a new thread by default
        rclpy.spin(Listener)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
