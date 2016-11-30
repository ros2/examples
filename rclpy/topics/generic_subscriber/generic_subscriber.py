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
import importlib

import rclpy


def subscriber_cb(msg):
    print('received: %r\n' % msg)


def subscriber(message_type):
    separator_idx = message_type.find('/')
    message_package = message_type[:separator_idx]
    message_name = message_type[separator_idx+1:]
    module = importlib.import_module(message_package + '.msg')
    msg_mod = getattr(module, message_name)

    rclpy.init([])

    node = rclpy.create_node('subscriber_%s_%s' % (message_package, message_name))

    node.create_subscription(
        msg_mod, 'topic_%s_%s' % (message_package, message_name), subscriber_cb)

    print('subscriber: beginning loop')
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('message_type', nargs='?', default='std_msgs/String',
                        help='type of the ROS message')
    args = parser.parse_args()
    try:
        subscriber(
            message_type=args.message_type)
    except KeyboardInterrupt:
        print('subscriber stopped cleanly')
    except:
        raise
