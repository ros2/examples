#!/usr/bin/env python3

import rclpy
from simple_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.Node("talker")
    pub = node.create_publisher("chatter", String, 10)

    rate = rclpy.WallRate(2)
    count = 0

    while rclpy.ok():
        msg = String("Hello World: {0}".format(count))
        count += 1
        pub.publish(msg)

        rclpy.spin_some(node)
        rate.sleep()

if __name__ == '__main__':
    main()
