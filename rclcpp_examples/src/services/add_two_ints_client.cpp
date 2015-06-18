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

#include <example_interfaces/srv/add_two_ints.hpp>

void on_result(rclcpp::client::Client<example_interfaces::srv::AddTwoInts>::SharedFuture cb_f)
{
  std::cout << "Result of add_two_ints: " << cb_f.get()->sum << std::endl;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_client");

  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 2;
  request->b = 3;

  client->async_send_request(request, on_result);
  rclcpp::spin(node);

  return 0;
}
