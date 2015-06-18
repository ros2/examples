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

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <std_interfaces/msg/string.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("talker");

  auto chatter_pub = node->create_publisher<std_interfaces::msg::String>("chatter", 7);

  rclcpp::WallRate loop_rate(2);

  auto msg = std::make_shared<std_interfaces::msg::String>();
  auto i = 1;

  while (rclcpp::ok()) {
    msg->data = "Hello World: " + std::to_string(i++);
    std::cout << "Publishing: '" << msg->data << "'" << std::endl;
    chatter_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
