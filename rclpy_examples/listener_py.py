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

import argparse
import os
import sys

import rclpy
from rclpy.qos import qos_profile_default

from std_msgs.msg import String


def chatter_callback(msg):
    print('I heard: [%s]' % msg.data)


def main(args=None):
    if args is None:
        args = sys.argv
    else:
        if len(args) > 1:
            from rclpy.impl.rmw_implementation_tools import select_rmw_implementation
            select_rmw_implementation(args[1])

    rclpy.init()

    node = rclpy.create_node('listener')

    sub = node.create_subscription(
        String, 'chatter', chatter_callback, qos_profile_default)
    assert sub  # prevent unused warning

    while rclpy.ok():
        rclpy.spin_once(node)


class MainForRmwImpl(object):
    def __getattr__(self, key):
        return main([os.path.basename(__file__), key])


main_for_rmw_impl = MainForRmwImpl()

if __name__ == '__main__':
    from rclpy.impl.rmw_implementation_tools import get_rmw_implementations
    rmw_implementations = get_rmw_implementations()
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('rmw_implementation', default=rmw_implementations[0],
                        choices=rmw_implementations,
                        help='rmw_implementation identifier')
    args = parser.parse_args()
    main(args)
