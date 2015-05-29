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

#include <simple_msgs/msg/all_builtin_types.hpp>
#include <simple_msgs/msg/all_dynamic_array_types.hpp>
#include <simple_msgs/msg/all_primitive_types.hpp>
#include <simple_msgs/msg/all_static_array_types.hpp>
#include <simple_msgs/msg/nested.hpp>
#include <simple_msgs/msg/string.hpp>
#include <simple_msgs/msg/uint32.hpp>

#include <userland_msgs/srv/add_two_ints.hpp>

void add(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<userland_msgs::srv::AddTwoInts::Request> request,
  std::shared_ptr<userland_msgs::srv::AddTwoInts::Response> response)
{
  std::cout << "Incoming request" << std::endl;
  std::cout << "a: " << request->a << " b: " << request->b << std::endl;
  response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_server");

  node->create_service<userland_msgs::srv::AddTwoInts>("add_two_ints", add);

  rclcpp::spin(node);

  return 0;
}
