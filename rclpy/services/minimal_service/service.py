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

import rclpy

from example_interfaces.srv import AddTwoInts


def add_two_ints_callback(request, response):
    response.sum = request.a + request.b
    print('Incoming request\na: %d b: %d' % (request.a, request.b))

    return response


def main(args=None):
    rclpy.init(args)

    node = rclpy.create_node('add_two_ints_server')

    srv = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)
    while rclpy.ok():
        rclpy.spin_once(node)

    # Destroy the service attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_service(srv)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
