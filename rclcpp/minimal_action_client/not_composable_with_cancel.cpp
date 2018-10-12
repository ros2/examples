// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <inttypes.h>
#include <memory>
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_action_client");
  auto action_client = rclcpp_action::create_action_client<Fibonacci>(node, "fibonacci");

  // Populate a goal
  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 10;

  // Send goal and wait for result (registering feedback callback is optional)
  auto goal_handle = action_client->async_send_goal(goal_msg);
  auto wait_result = rclcpp::spin_until_future_complete
    node,
    goal_handle->result_future(),
    std::chrono::duration<int64_t, std::milli>(3000));

  if (rclcpp::executor::FutureReturnCode::TIMEOUT == wait_result)
  {
    RCLCPP_INFO(node->get_logger(), "canceling goal");
    // Cancel the goal since it is taking too long
    auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
    if (rclcpp::spin_until_future_complete(node, cancel_result_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "failed to cancel goal");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "goal canceled successfully");
    rclcpp::shutdown();
    return 0;
  }
  else if (rclcpp::executor::FutureReturnCode::SUCCESS != wait_result)
  {
    RCLCPP_ERROR(node->get_logger(), "failed to get result");
    rclcpp::shutdown();
    return 1;
  }

  auto result = goal_handle->result_future().get();
  RCLCPP_INFO(node->get_logger(), "result received");
  rclcpp::shutdown();
  return 0;
}
