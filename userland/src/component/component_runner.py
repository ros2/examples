#!/usr/bin/env python3

import rclpy


def main(options):
    context = rclpy.get_default_context()
    component = rclpy.get_component(options.package, options.component)(context)
    assert component.has_inited is True, "User did not call rclpy.Node's __init__"
    executor = rclpy.SingleThreadedExecutor()
    executor.add_node(component)
    executor.spin()

if __name__ == '__main__':
    main()
