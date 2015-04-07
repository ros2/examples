// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <simple_msgs/Vector3Int.h>
#include "userland/command_line_arguments.h"


template<typename T>
int publish(rclcpp::Node::SharedPtr node, void (* set_data_func)(typename T::Ptr &, size_t))
{
  auto p = node->create_publisher<T>("imu", 1000);
  typename T::Ptr ros_msg(new T());

  auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(10);
  size_t i = 1;
  while (rclcpp::ok()) {
    set_data_func(ros_msg, i);
    p->publish(ros_msg);
    std::cout << "published Vector3Int ROS msg #" << i << std::endl;
    ++i;
    rate.sleep();
  }
  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<float> diff = (end - start);
  std::cout << "Runtime: " << diff.count() << " seconds" << std::endl;

  return 0;
}

void set_accel_data(simple_msgs::Vector3Int::Ptr & ros_msg, size_t i)
{
  ros_msg->x = 1;
  ros_msg->y = 2;
  ros_msg->z = 3;
}

template<typename T>
void set_empty(typename T::Ptr & ros_msg, size_t i)
{
  ros_msg.reset(new T());
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (has_argument(argv, argv + argc, "--help")) {
    std::cout << "usage: " << argv[0] << std::endl;
    print_message_usage();
    return 0;
  }

  auto node = rclcpp::Node::make_shared("publisher");
  return publish<simple_msgs::Vector3Int>(node, &set_accel_data);
}
