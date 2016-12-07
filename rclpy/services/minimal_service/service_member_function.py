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


class MinimalService:
    def __init__(self, node):
        self.srv = node.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        print('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args)

    node = rclpy.create_node('add_two_ints_server')
    minimal_service = MinimalService(node)
    minimal_service  # prevent unused variable warning

    while rclpy.ok():
        rclpy.spin_once(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
