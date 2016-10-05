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
from rclpy.qos import qos_profile_default, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String


def chatter_callback(msg):
    print('I heard: [%s]' % msg.data)


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args)

    custom_qos_profile = qos_profile_default
    custom_qos_profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_BEST_EFFORT
    custom_qos_profile.history = QoSHistoryPolicy.RMW_QOS_POLICY_KEEP_LAST_HISTORY

    node = rclpy.create_node('listener')
  
    sub = node.create_subscription(String, 'chatter', chatter_callback, custom_qos_profile)

    assert sub  # prevent unused warning

    while rclpy.ok():
        rclpy.spin_once(node)

if __name__ == '__main__':
    main()
