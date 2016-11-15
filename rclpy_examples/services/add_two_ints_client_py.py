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


def main(args=None):
    rclpy.init(args)

    node = rclpy.create_node('add_two_ints_client')

    cli = node.create_client(AddTwoInts, 'add_two_ints')
    max_iter = 3
    i = 0
    while rclpy.ok() and i < max_iter:
        req = AddTwoInts.Request()
        req.a = i
        req.b = i + 1
        cli.call(req)
        cli.wait_for_future()
        print('Result of add_two_ints: %d' % cli.response.sum)
        i += 1

if __name__ == '__main__':
    main()
