#!/usr/bin/env python3

# Copyright 2015 Open Source Robotics Foundation, Inc.
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
from simple_msgs.msg import String


class Talker(rclpy.Node):
    def __init__(self, context):
        rclpy.Node.__init__(self, "talker", context)
        self.chatter_pub = self.create_publisher("chatter", String, 10)
        self.publisher_timer = self.create_wall_timer(0.5, self.on_timer)
        self.count = 0

    def on_timer(self):
        msg = String("Hello World: {0}".format(self.count))
        self.count += 1
        self.chatter_pub.publish(msg)
