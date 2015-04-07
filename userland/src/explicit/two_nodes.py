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


def on_message(msg):
    print("I heard: [{0}]".format(msg.data))


def on_timer(publisher, i):
    msg = String("Hello World: {0}".format(i))
    i += 1
    publisher.publish(msg)


def main():
    rclpy.init()
    node1 = rclpy.create_node("node1")
    node2 = rclpy.create_node("node2")

    pub = node1.create_publisher("chatter", String, 10)
    node2.create_subscription("chatter", 10, on_message)

    i = 0

    def call_on_timer():
        return on_timer(pub, i)

    node1.create_timer(0.5, call_on_timer)

    executor = rclpy.SingleThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    executor.spin()

if __name__ == '__main__':
    main()
