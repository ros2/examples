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
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>


rclcpp_action::GoalResponse handle_goal(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  // Let's reject sequences that are over 9000
  if (goal_handle->goal.order > 9000)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT;
}

rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::ResultResponse<Fibonacci::Result> execute(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  rclcpp::Rate loop_rate(1);
  const auto& goal = goal_handle->goal;
  auto feedback = Fibonacci::Feedback();
  auto& sequence = feedback.sequence;
  sequence.push_back(0);
  sequence.push_back(1);
  auto result_response = rclcpp_action::ResultResponse<Fibonacci::Result>();

  for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
  {
    // Check if there is a cancel request
    if (goal_handle->is_cancel_request())
    {
      result_response.response = rclcpp_action::ResultResponse::CANCELED;
      result_response.result.sequence = sequence;
      result_response.message = "Canceling goal at clients request.";
      return result_response;
    }

    // Check if goal is done
    if (i > goal->order)
    {
      result_response.response = rclcpp_action::ResultResponse::SUCCEEDED;
      result_response.result.sequence = sequence;
      result_response.message = "Successfully computed Fibonacci sequence.";
      return result_response;
    }
    // Update sequence
    sequence.push_back(sequence[i] + sequence[i - 1]);
    // Publish feedback
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_action_server");

  // Create an action server with three callbacks
  //   'handle_goal' and 'handle_cancel' are called by the Executor (rclcpp::spin)
  //   'execute' is called whenever 'handle_goal' returns by accepting a goal
  //    Calls to 'execute' are made in an available thread from a pool of four.
  auto action_server = rclcpp_action::create_action_server<Fibonacci>(
    node,
    "fibonacci",
    handle_goal,
    handle_cancel,
    execute,
    rclcpp_action::StaticThreadPoolExecutePolicy(4));

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
