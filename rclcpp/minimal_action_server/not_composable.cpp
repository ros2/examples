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

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp::GoalHandle<Fibonacci>
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::ActionServer<Fibonacci>::SharedPtr g_action_server = nullptr;
std::shared_ptr<GoalHandleFibonacci> g_goal_handle = nullptr;
std::shared_ptr<rFibonacci::Feedback> g_feedback = nullptr;


void handle_goal(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  // Only allow one goal active at a time
  if (g_goal_handle && g_goal_handle->is_processing())
  {
    auto result = Fibonacci::Result();
    result.sequence = g_feedback->sequence;
    g_goal_handle->set_canceled(result, "New goal received, canceled previous goal.");
  }
  goal_handle->accept();
  g_goal_handle = goal_handle
  // Initialize Fibonacci sequence
  g_feedback->sequence.clear();
  g_feedback->sequence.push_back(0);
  g_feedback->sequence.push_back(1);
}

void handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  if (g_goal_handle && (*g_goal_handle == *goal_handle))
  {
    auto result = Fibonacci::Result();
    result.sequence = g_feedback->sequence;
    goal_handle->set_canceled(result, "Canceling goal at clients request.");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("minimal_action_server");
  g_action_server = g_node->create_action_server<Fibonacci>("fibonacci", handle_goal, handle_cancel);
  g_feedback = std::make_shared<Fibonacci::Feedback>(Fibonacci::Feedback());

  rclcpp::Rate loop_rate(1);
  while (rclcpp::ok())
  {
    // Process an active goal
    if (g_goal_handle && g_goal_handle->is_active())
    {
      const auto& goal = g_goal_handle->goal;
      const auto& sequence = g_feedback->sequence;
      const int i = sequence.size() - 1;
      // Check if goal is done
      if (i > goal->order)
      {
        auto result = Fibonacci::Result();
        result.sequence = sequence;
        goal->set_succeeded(result);
      }
      else
      {
        // Update sequence
        sequence.push_back(sequence[i] + sequence[i - 1]);
        // Publish feedback
        g_goal_handle->publish_feedback(g_feedback);
      }
    }
    rclcpp::spin_some(g_node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  g_feedback = nullptr;
  g_action_server = nullptr;
  g_node = nullptr;
  return 0;
}
