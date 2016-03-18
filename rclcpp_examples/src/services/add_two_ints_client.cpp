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
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"

// TODO(wjwwood): make this into a method of rclcpp::client::Client.
example_interfaces::srv::AddTwoInts_Response::SharedPtr send_request(
  rclcpp::Node::SharedPtr node,
  rclcpp::client::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client,
  example_interfaces::srv::AddTwoInts_Request::SharedPtr request)
{
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return result.get();
  } else {
    return NULL;
  }
}

// This function could be considered a prototype of an API that should be
// promoted into rclcpp. This version polls; doing it in an event-driven
// manner would be better.
bool wait_for_service(rclcpp::Node::SharedPtr node,
  const std::string & topic,
  const std::chrono::steady_clock::duration timeout)
{
  const std::chrono::milliseconds sleep_per_loop(1);
  auto end = std::chrono::steady_clock::now() + timeout;
  bool success = false;
  bool printed = false;
  while (std::chrono::steady_clock::now() < end) {
    try {
      if (node->count_subscribers(topic) == 0) {
        success = true;
        break;
      }
    } catch (std::runtime_error & e) {
      // fastrtps doesn't yet implement rmw_count_subscribers(), but rather
      // throws an exception instead. We'll just run out the clock and
      // return true, hoping for the best.
      if (!printed) {
        fprintf(stderr, "count_subscribers() exception (this is expected for FastRTPS): %s\n",
          e.what());
        printed = true;
      }
      rmw_reset_error();
      success = true;
    }
    auto remaining = end - std::chrono::steady_clock::now();
    if (remaining < sleep_per_loop) {
      if (remaining > std::chrono::steady_clock::duration::zero()) {
        std::this_thread::sleep_for(remaining);
      }
    } else {
      std::this_thread::sleep_for(sleep_per_loop);
    }
  }
  return success;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_client");

  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 2;
  request->b = 3;

  if (!wait_for_service(node, "add_two_ints", std::chrono::seconds(2))) {
    printf("add_two_ints service not available. Exiting.\n");
    return 1;
  }

  // TODO(wjwwood): make it like `client->send_request(node, request)->sum`
  // TODO(wjwwood): consider error condition
  auto result = send_request(node, client, request);
  if (result) {
    printf("Result of add_two_ints: %zd\n", result->sum);
  } else {
    printf("add_two_ints_client was interrupted. Exiting.\n");
  }

  return 0;
}
