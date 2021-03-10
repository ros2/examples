# Copyright 2020 Evan Flynn
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

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class PointCloudPublisher(Node):

    rate = 30
    moving = True
    width = 100
    height = 100

    header = Header()
    header.frame_id = 'map'

    dtype = PointField.FLOAT32
    point_step = 16
    fields = [PointField(name='x', offset=0, datatype=dtype, count=1),
              PointField(name='y', offset=4, datatype=dtype, count=1),
              PointField(name='z', offset=8, datatype=dtype, count=1),
              PointField(name='intensity', offset=12, datatype=dtype, count=1)]

    def __init__(self):
        super().__init__('pc_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'test_cloud', 10)
        timer_period = 1 / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.header.stamp = self.get_clock().now().to_msg()
        x, y = np.meshgrid(np.linspace(-2, 2, self.width), np.linspace(-2, 2, self.height))
        z = 0.5 * np.sin(2*x-self.counter/10.0) * np.sin(2*y)
        points = np.array([x, y, z, z]).reshape(4, -1).T
        pc2_msg = point_cloud2.create_cloud(self.header, self.fields, points)
        self.publisher_.publish(pc2_msg)

        if self.moving:
            self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    pc_publisher = PointCloudPublisher()
    rclpy.spin(pc_publisher)
    pc_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
