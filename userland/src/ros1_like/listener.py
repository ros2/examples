#!/usr/bin/env python3

import rclpy


def callback(msg):
    print("I heard: [{0}]".format(msg.data))


def main():
    rclpy.init()
    node = rclpy.Node("listener")

    node.create_subscription("chatter", callback, 10)

    executor = rclpy.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
