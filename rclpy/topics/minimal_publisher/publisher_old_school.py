# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from time import sleep

import rclpy

from std_msgs.msg import String


def main(args=None):
    rclpy.init(args)

    node = rclpy.create_node('minimal_publisher')

    publisher = node.create_publisher(String, 'topic')

    msg = String()

    i = 0
    while rclpy.ok():
        msg.data = 'Hello World: %d' % i
        i += 1
        print('Publishing: "%s"' % msg.data)
        publisher.publish(msg)
        # For now spin_once cannot be called in nodes containing only publishers
        # This is not a guideline and will be possible in the near future
        sleep(0.5)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
