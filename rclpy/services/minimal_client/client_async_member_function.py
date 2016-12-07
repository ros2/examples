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

import time

import rclpy

from example_interfaces.srv import AddTwoInts


class MinimalClientAsync:
    def __init__(self, node):
        self.cli = node.create_client(AddTwoInts, 'add_two_ints')
        # TODO(mikaelarguedas) remove this once wait for service implemented
        # wait for connection to be established
        # (no wait for service in Python yet)
        time.sleep(1)
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        self.cli.call(self.req)


def main(args=None):
    rclpy.init(args)

    node = rclpy.create_node('minimal_client_async')

    minimal_client = MinimalClientAsync(node)
    minimal_client.send_request()

    while rclpy.ok():
        # TODO(mikaelarguedas) This is not the final API, and this does not scale
        # for multiple pending requests. This will change once an executor model is implemented
        # In the future the response will not be stored in cli.response
        if minimal_client.cli.response is not None:
            print(
                'Result of add_two_ints: for %d + %d = %d' %
                (minimal_client.req.a, minimal_client.req.b, minimal_client.cli.response.sum))
            break
        rclpy.spin_once(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
