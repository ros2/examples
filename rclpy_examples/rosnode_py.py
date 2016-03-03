# Copyright 2016 Erle Robotics, LLC.
#
# A relevant part of the code has been written taking inpiration
# from ROS 1 ros_comm package
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

import rclpy.rosnode as rosnode


def main(args=None):

    # TODO discuss whether rclpy should or shouldn't be initialized here
    #rclpy.init(argv)
    rosnode.rosnodemain()


if __name__ == '__main__':
    main()
