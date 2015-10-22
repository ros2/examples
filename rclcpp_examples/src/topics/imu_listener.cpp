// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  printf(" accel: [%+6.3f %+6.3f %+6.3f]\n",
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("imu_listener");
  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "imu", imu_cb, rmw_qos_profile_sensor_data);
  rclcpp::spin(node);
  return 0;
}
