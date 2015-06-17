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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simple_msgs/msg/vector3.hpp>
#include "userland/command_line_arguments.h"


template<typename T>
rclcpp::subscription::SubscriptionBase::SharedPtr subscribe(
  rclcpp::Node::SharedPtr node,
  typename rclcpp::subscription::Subscription<T>::CallbackType callback)
{
  auto sub = node->create_subscription<T>("imu", 1000, callback);
  return sub;
}

void print_accel_data(const simple_msgs::msg::Vector3::ConstSharedPtr & msg)
{
  std::cout << "-------------------------" << std::endl;
  std::cout << "Got accel x=" << msg->x << std::endl;
  std::cout << "Got accel y=" << msg->y << std::endl;
  std::cout << "Got accel z=" << msg->z << std::endl;
  std::cout << "-------------------------" << std::endl;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (has_argument(argv, argv + argc, "--help")) {
    std::cout << "usage:" << argv[0] << std::endl;
    print_message_usage();
    return 0;
  }

  auto node = rclcpp::Node::make_shared("subscriber");
  rclcpp::subscription::SubscriptionBase::SharedPtr sub;

  sub = subscribe<simple_msgs::msg::Vector3>(node, print_accel_data);
  rclcpp::spin(node);

  return 0;
}
