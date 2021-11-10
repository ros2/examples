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

import sys
import threading

import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class ThrottledCallbackGroup(CallbackGroup):
    """
    Throttle callbacks using a token bucket.

    Callback groups are responsible for controlling when callbacks are allowed to be executed.
    rclpy provides two groups: one which always allows a callback to be executed, and another which
    allows only one callback to be executed at a time. If neither of these are sufficient then a
    custom callback group should be used instead.
    """

    def __init__(self, node):
        super().__init__()
        self.timer = node.create_timer(0.5, self.timer_callback)
        self.bucket = 10
        self.bucket_max = 10
        self.lock = threading.Lock()

    def can_execute(self, entity):
        """
        Ask group if this entity could be executed.

        :param entity: A timer, subscriber, client, or service instance
        :rtype bool: true if a callback can be executed
        """
        return self.bucket > 0

    def beginning_execution(self, entity):
        """
        Get permission from the group to execute a callback for an entity.

        :param entity: A timer, subscriber, client, or service instance
        :rtype bool: true if the executor has permission to execute it
        """
        with self.lock:
            if self.bucket > 0:
                # Take a token
                self.bucket -= 1
                return True
            # The bucket has no tokens
            return False

    def ending_execution(self, entity):
        """
        Notify group that a callback finished executing.

        :param entity: A timer, subscriber, client, or service instance
        """
        pass

    def timer_callback(self):
        """Replenish the tokens in the bucket at a steady rate."""
        with self.lock:
            # If there is room in the bucket, add a token to it.
            if self.bucket < self.bucket_max:
                self.bucket += 1


class ThrottledTalker(Node):
    """A Node which uses a custom callback group."""

    def __init__(self):
        super().__init__('intermittent_talker')
        self.i = 0
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.group = ThrottledCallbackGroup(self)
        # Timer triggers very quickly, but is part of a throttled group
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.group)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        talker = ThrottledTalker()
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        talker.destroy_node()


if __name__ == '__main__':
    main()
