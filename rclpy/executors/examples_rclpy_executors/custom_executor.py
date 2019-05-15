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

from concurrent.futures import ThreadPoolExecutor
import os

from examples_rclpy_executors.listener import Listener
from examples_rclpy_executors.talker import Talker
import rclpy
from rclpy.executors import Executor
from rclpy.node import Node
from std_msgs.msg import String


class Estopper(Node):

    def __init__(self):
        super().__init__('estopper')
        self.sub = self.create_subscription(String, 'estop', self.estop_callback, 10)

    def estop_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


class PriorityExecutor(Executor):
    """
    Execute high priority callbacks in multiple threads, all others in a single thread.

    This is an example of a custom exectuor in python. Executors are responsible for managing
    how callbacks get mapped to threads. Rclpy provides two executors: one which runs all callbacks
    in the main thread, and another which runs callbacks in a pool of threads. A custom executor
    should be written if neither are appropriate for your application.
    """

    def __init__(self):
        super().__init__()
        self.high_priority_nodes = set()
        self.hp_executor = ThreadPoolExecutor(max_workers=os.cpu_count() or 4)
        self.lp_executor = ThreadPoolExecutor(max_workers=1)

    def add_high_priority_node(self, node):
        self.high_priority_nodes.add(node)
        # add_node inherited
        self.add_node(node)

    def spin_once(self, timeout_sec=None):
        """
        Execute a single callback, then return.

        This is the only function which must be overridden by a custom executor. Its job is to
        start executing one callback, then return. It uses the method `wait_for_ready_callbacks`
        to get work to execute.

        :param timeout_sec: Seconds to wait. Block forever if None. Don't wait if <= 0
        :type timeout_sec: float or None
        """
        # wait_for_ready_callbacks yields callbacks that are ready to be executed
        try:
            handler, group, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except StopIteration:
            pass
        else:
            if node in self.high_priority_nodes:
                self.hp_executor.submit(handler)
            else:
                self.lp_executor.submit(handler)


def main(args=None):
    rclpy.init(args=args)
    try:
        listener = Listener()
        talker = Talker()
        estopper = Estopper()

        executor = PriorityExecutor()
        executor.add_high_priority_node(estopper)
        executor.add_node(listener)
        executor.add_node(talker)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            estopper.destroy_node()
            talker.destroy_node()
            listener.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
