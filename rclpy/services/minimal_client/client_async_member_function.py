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
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        self.cli.call(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        # TODO(mikaelarguedas) This is not the final API, and this does not scale
        # for multiple pending requests. This will change once an executor model is implemented
        # In the future the response will not be stored in cli.response
        if minimal_client.cli.response is not None:
            minimal_client.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' %
                (minimal_client.req.a, minimal_client.req.b, minimal_client.cli.response.sum))
            break
        rclpy.spin_once(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
