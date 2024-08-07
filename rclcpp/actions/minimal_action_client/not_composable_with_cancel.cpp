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

#include <chrono>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_action_client");
  auto action_client = rclcpp_action::create_client<Fibonacci>(node, "fibonacci");

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  // Populate a goal
  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 10;

  RCLCPP_INFO(node->get_logger(), "Sending goal");
  // Send goal and wait for result (registering feedback callback is optional)
  auto goal_handle_future = action_client->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);

  auto wait_result = rclcpp::spin_until_future_complete(
    node,
    result_future,
    std::chrono::seconds(3));

  if (rclcpp::FutureReturnCode::TIMEOUT == wait_result) {
    RCLCPP_INFO(node->get_logger(), "canceling goal");
    // Cancel the goal since it is taking too long
    auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
    if (rclcpp::spin_until_future_complete(node, cancel_result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "failed to cancel goal");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "goal is being canceled");
  } else if (rclcpp::FutureReturnCode::SUCCESS != wait_result) {
    RCLCPP_ERROR(node->get_logger(), "failed to get result");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult wrapped_result = result_future.get();
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      return 1;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      return 1;
  }

  rclcpp::shutdown();
  return 0;
}
