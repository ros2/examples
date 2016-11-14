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

import sys

import rclpy

from rclpy.qos import qos_profile_default

from std_msgs.msg import String


i = 1
data = ''


def timer_cb():
    global i, data

    data = 'Hello World: {0}'.format(i)
    i += 1


def main(args=None):
    global i, data
    if args is None:
        args = sys.argv

    rclpy.init(args)

    node = rclpy.create_node('talker')

    chatter_pub = node.create_publisher(String, 'chatter', qos_profile_default)
    timer = node.create_timer(1, timer_cb)
    timer  # unused
    msg = String()

    while True:
        if i < 17:
            msg.data = data
            print('Publishing: "{0}"'.format(msg.data))
            chatter_pub.publish(msg)
        elif i == 17:
            print('destroying publisher')
            node.destroy_publisher(chatter_pub)
            node.destroy_timer(timer)
        if i == 3:
            print('changing time period')
            print('period before: %d' % timer.timer_period_ns)
            timer.timer_period_ns = 0.33 * 1000 * 1000 * 1000
            print('period after: %d' % timer.timer_period_ns)
        rclpy.spin_once(node, 2)

if __name__ == '__main__':
    main()
