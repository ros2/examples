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
#include <simple_msgs/msg/imu.hpp>
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
    std::cout << "published Imu ROS msg #" << i << std::endl;
    ++i;
    rate.sleep();
  }
  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<float> diff = (end - start);
  std::cout << "Runtime: " << diff.count() << " seconds" << std::endl;

  return 0;
}

void set_imu_data(simple_msgs::msg::Imu::Ptr & ros_msg, size_t i)
{
  // Define the header
  ros_msg->header.seq = i;
  ros_msg->header.stamp.sec = 0;
  ros_msg->header.stamp.nanosec = 0;
  ros_msg->header.frame_id = "";

  // Define the orientation
  ros_msg->orientation.x = 0;
  ros_msg->orientation.y = 0;
  ros_msg->orientation.z = 0;
  ros_msg->orientation.w = 0;


  // Define the orientation_covariance
  for (int i = 0; i < 9; i++) {
    ros_msg->orientation_covariance[i] = 0;
  }
  // Define the angular_velocity
  ros_msg->angular_velocity.x = 0;
  ros_msg->angular_velocity.y = 0;
  ros_msg->angular_velocity.z = 0;

  // Define the angular_velocity_covariance
  for (int i = 0; i < 9; i++) {
    ros_msg->angular_velocity_covariance[i] = 0;
  }
  // Define the linear_acceleration
  ros_msg->linear_acceleration.x = 1 + i;
  ros_msg->linear_acceleration.y = 2 + i;
  ros_msg->linear_acceleration.z = 3 + i;

  // Define the linear_acceleration_covariance
  for (int i = 0; i < 9; i++) {
    ros_msg->linear_acceleration_covariance[i] = 0;
  }
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
  return publish<simple_msgs::msg::Imu>(node, &set_imu_data);
}
