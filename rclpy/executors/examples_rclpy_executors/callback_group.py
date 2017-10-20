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

from examples_rclpy_executors.listener import Listener
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import rclpy.logging
from std_msgs.msg import String

logger = rclpy.logging.get_named_logger('callback_group_example')
# logger.set_severity_threshold(rclpy.logging.LoggingSeverity.DEBUG)


class DoubleTalker(rclpy.Node):
    """Publish messages to a topic using two publishers at different rates."""

    def __init__(self, **node_init_args):
        super().__init__('double_talker', **node_init_args)

        self.i = 0
        self.pub = self.create_publisher(String, 'chatter')

        # This type of callback group only allows one callback to be executed at a time
        self.group = MutuallyExclusiveCallbackGroup()
        # Pass the group as a parameter to give it control over the execution of the timer callback
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=self.group)
        self.timer2 = self.create_timer(0.5, self.timer_callback, callback_group=self.group)
        self.logger.debug('Initialization complete')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        self.logger.info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        # MultiThreadedExecutor executes callbacks with a thread pool. If num_threads is not
        # specified then num_threads will be multiprocessing.cpu_count() if it is implemented.
        # Otherwise it will use a single thread. This executor will allow callbacks to happen in
        # parallel, however the MutuallyExclusiveCallbackGroup in DoubleTalker will only allow its
        # callbacks to be executed one at a time. The callbacks in Listener are free to execute in
        # parallel to the ones in DoubleTalker however.
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(DoubleTalker(parent_logger=logger))
        executor.add_node(Listener(parent_logger=logger))
        logger.debug('Nodes added to executor')
        try:
            executor.spin()
        finally:
            logger.debug('Executor shutting down')
            executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
