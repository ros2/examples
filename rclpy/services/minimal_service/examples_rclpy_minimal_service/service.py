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

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.executors import ExternalShutdownException

g_node = None


def add_two_ints_callback(request, response):
    global g_node
    response.sum = request.a + request.b
    g_node.get_logger().info(
        'Incoming request\na: %d b: %d' % (request.a, request.b))

    return response


def main(args=None):
    global g_node

    try:
        with rclpy.init(args=args):
            g_node = rclpy.create_node('minimal_service')

            srv = g_node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)
            srv  # Quiet flake8 warnings about unused variable
            while rclpy.ok():
                rclpy.spin_once(g_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
