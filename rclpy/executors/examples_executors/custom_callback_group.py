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


class ThrottledCallbackGroup(rclpy.CallbackGroup):
    '''Demonstrate an example of a custom group.

    This groups throttles callbacks using a token bucket
    '''

    def __init__(self):
        super().__init__(self)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.bucket = 0
        self.bucket_max = 10

    def can_take_callback(self, callback):
        '''Return true if a callback can be executed.'''
        # assumes caller acquired the group lock already
        return self.bucket > 0

    def take_callback(self, callback):
        # assumes caller acquired the group lock already
        self.bucket -= 1
        return super().take_callback(callback)

    def timer_callback(self):
        with self.lock:
            if self.bucket < self.bucket_max:
                self.bucket += 1


class ThrottledTalkerListener(rclpy.Node):
    def __init__(self):
        super().__init__('intermittent_talker_listener')

        self.pub = self.create_publisher(String, 'chatter')
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.group = rclpy.ThrottledCallbackGroup()
        self.sub = self.create_subscription(
            String, 'chatter', self.chatter_callback, group=self.group)

    def chatter_callback(self, msg):
        print('I heard: [%s]' % msg.data)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        print('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(ThrottledTalkerListener())
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
