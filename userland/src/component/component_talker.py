#!/usr/bin/env python3

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
