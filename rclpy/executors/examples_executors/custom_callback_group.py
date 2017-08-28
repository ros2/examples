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

import threading

import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from std_msgs.msg import String


class ThrottledCallbackGroup(CallbackGroup):
    """Demonstrate an example of a custom group.

    This groups throttles callbacks using a token bucket
    """

    def __init__(self, node):
        super().__init__()
        self.timer = node.create_timer(0.5, self.timer_callback)
        self.bucket = 10
        self.bucket_max = 10
        self.lock = threading.Lock()

    def can_execute(self, entity):
        """Return true if a callback can be executed."""
        return self.bucket > 0

    def begin_execution(self, entity):
        with self.lock:
            if self.bucket > 0:
                self.bucket -= 1
                return True
            return False

    def end_execution(self, entity):
        pass

    def timer_callback(self):
        with self.lock:
            if self.bucket < self.bucket_max:
                self.bucket += 1


class ThrottledTalkerListener(Node):
    def __init__(self):
        super().__init__('intermittent_talker_listener')
        self.i = 0
        self.pub = self.create_publisher(String, 'chatter')
        self.group = ThrottledCallbackGroup(self)
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.group)
        self.sub = self.create_subscription(String, 'chatter', self.chatter_callback)

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
