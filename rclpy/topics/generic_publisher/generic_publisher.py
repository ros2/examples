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
import json
import time

import rclpy


def publisher(message_type, values):
    separator_idx = message_type.find('/')
    message_package = message_type[:separator_idx]
    message_name = message_type[separator_idx + 1:]
    module = importlib.import_module(message_package + '.msg')
    msg_mod = getattr(module, message_name)
    values_dictionary = json.loads(values)

    rclpy.init([])

    node = rclpy.create_node('publisher_%s_%s' % (message_package, message_name))

    chatter_pub = node.create_publisher(
        msg_mod, 'topic_%s_%s' % (message_package, message_name))

    msg = msg_mod()
    for field_name, field_value in values_dictionary.items():
        field_type = type(getattr(msg, field_name))
        setattr(msg, field_name, field_type(field_value))

    print('publisher: beginning loop')
    while rclpy.ok():
        chatter_pub.publish(msg)
        print('publishing %r\n' % msg)
        time.sleep(1)
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('message_type', nargs='?', default='std_msgs/String',
                        help='type of the ROS message')
    parser.add_argument('values', nargs='?', default='{}',
                        help='values to fill the message with, in json format.' +
                        'e.g. {"data": "Hello World"}')
    args = parser.parse_args()
    try:
        publisher(
            message_type=args.message_type,
            values=args.values)
    except KeyboardInterrupt:
        print('publisher stopped cleanly')
    except:
        raise
