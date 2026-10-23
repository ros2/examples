import rclpy
from rclpy.node import Node


class MinimalTimer(Node):

    def __init__(self):
        super().__init__('minimal_timer')

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Timer callback triggered')


def main(args=None):
    rclpy.init(args=args)

    minimal_timer = MinimalTimer()

    rclpy.spin(minimal_timer)

    minimal_timer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
