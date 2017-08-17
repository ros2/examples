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

from examples_executors.listener import Listener
import rclpy
from std_msgs.msg import String


class DoubleTalker(rclpy.Node):
    def __init__(self):
        super().__init__('double_talker')

        self.i = 0
        self.pub = self.create_publisher(String, 'chatter')

        self.group = rclpy.MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(1.0, self.timer_callback, group=self.group)
        self.timer2 = self.create_timer(0.5, self.timer_callback, group=self.group)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        print('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        # MultiThreadedExecutor spawns every callback on a new thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(DoubleTalker())
        executor.add_node(Listener())
        executor.spin()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
