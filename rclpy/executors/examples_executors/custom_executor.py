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

from examples_executors.listener import Listener
from examples_executors.talker import Talker
import rclpy
from rclpy.executors import Executor
from rclpy.node import Node
from std_msgs.msg import String


class Estopper(Node):
    def __init__(self):
        super().__init__('estopper')
        self.sub = self.create_subscription(String, 'estop', self.estop_callback)

    def estop_callback(self, msg):
        print('I heard: [%s]' % msg.data)


class PriorityExecutor(Executor):
    """
    Execute high priority callbacks in multiple threads, all others in a signle thread.

    This is an example of a custom exectuor in python. Executors are responsible for managing
    how callbacks get mapped to threads.
    """

    def __init__(self):
        super().__init__()
        self.high_priority_nodes = set()
        self.low_priority_thread = None

    def add_high_priority_node(self, node):
        self.high_priority_nodes.add(node)
        # add_node inherited
        self.add_node(node)

    def can_run_low_priority(self):
        return self.low_priority_thread is None or not self.low_priority_thread.is_alive()

    def spin_once(self, timeout_sec=None):
        """
        Execute a single callback, then return.

        timeout_sec - seconds to wait for callbacks. Blocks forever if None. Don't wait if <= 0
        """
        # Wait only on high priority nodes if the low priority thread is taken.
        # this avoids spinning rapidly when low priority callbacks are available but
        # can't be acted on
        nodes = self.high_priority_nodes
        if self.can_run_low_priority():
            # get_nodes returns all nodes added to executor
            nodes = self.get_nodes()

        # wait_for_ready_callbacks yields callbacks that are ready to be executed
        for handler, group, node in self.wait_for_ready_callbacks(
                timeout_sec=timeout_sec, nodes=nodes):
                if node in self.high_priority_nodes:
                    t = threading.Thread(target=handler)
                    t.start()
                else:
                    self.low_priority_thread = threading.Thread(target=handler)
                    self.low_priority_thread.start()
                break


def main(args=None):
    rclpy.init(args=args)
    try:
        try:
            executor = PriorityExecutor()
            executor.add_high_priority_node(Estopper())
            executor.add_node(Listener())
            executor.add_node(Talker())
            executor.spin()
        finally:
            executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
