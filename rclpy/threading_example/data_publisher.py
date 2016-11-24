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
from std_msgs.msg import Int64


def main():
    rclpy.init()

    node = rclpy.create_node('data_publisher')
    data_pub = node.create_publisher(Int64, 'data')

    val = -1
    msg = Int64()

    # Function for publishing and updating data
    def update_data():
        nonlocal val
        val *= -1
        msg.data = val
        print('Publishing: "{0}"'.format(msg.data))
        data_pub.publish(msg)
        
    # Create a timer that will cause data to be published every 0.1 seconds
    node.create_timer(0.1, update_data)

    while rclpy.ok():
        rclpy.spin_once(node)


if __name__ == '__main__':
    main()
