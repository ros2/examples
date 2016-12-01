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
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_client");

  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      printf("add_two_ints_client was interrupted while waiting for the service. Exiting.\n");
      return 0;
    }
    printf("service not available, waiting again...\n");
  }

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 2;
  request->b = 3;

  auto future_result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, future_result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    printf("Result of add_two_ints: %zd\n", future_result.get()->sum);
  } else {
    printf("add_two_ints_client_async was interrupted. Exiting.\n");
  }

  rclcpp::shutdown();
  return 0;
}
