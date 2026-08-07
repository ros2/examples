// Copyright 2016 Open Source Robotics Foundation, Inc.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <functional>
#include <memory>
#include <thread>

#include "random_generator/action/randomizer.hpp"
#include "random_generator/visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace random_generator {
class RandomizerActionServer : public rclcpp::Node {
 public:
  using Randomizer = action::Randomizer;
  using GoalHandleRandomizer = rclcpp_action::ServerGoalHandle<Randomizer>;

  RANDOM_GENERATOR_PUBLIC
  explicit RandomizerActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("randomizer_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Randomizer>(
        this, "randomizer", std::bind(&RandomizerActionServer::handle_goal, this, _1, _2),
        std::bind(&RandomizerActionServer::handle_cancel, this, _1),
        std::bind(&RandomizerActionServer::handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(), "Started random generator action server");
  }

 private:
  rclcpp_action::Server<Randomizer>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const Randomizer::Goal> goal) {
    if (goal->min_val >= goal->max_val) {
      RCLCPP_ERROR(this->get_logger(),
                   "Received goal request with %d values between %f and %f. Min value must be "
                   "smaller than the Max value.",
                   goal->number_of_values, goal->min_val, goal->max_val);
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Received goal request with %d values between %f and %f",
                goal->number_of_values, goal->min_val, goal->max_val);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleRandomizer> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRandomizer> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RandomizerActionServer::execute, this, _1), goal_handle}.detach();
  }

  double random() { return ((long double)rand()) / ((long double)RAND_MAX); }

  void execute(const std::shared_ptr<GoalHandleRandomizer> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Randomizer::Feedback>();
    auto result = std::make_shared<Randomizer::Result>();

    auto values = result->random_values;
    for (feedback->number_of_random_values_calculated = 0;
         feedback->number_of_random_values_calculated < goal->number_of_values && rclcpp::ok();) {
      rclcpp::Rate loop_rate(5 * (random() + 0.1));
      loop_rate.sleep();
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Get a new value
      feedback->new_random_value = goal->min_val + (random()) * (goal->max_val - goal->min_val);
      result->random_values.push_back(feedback->new_random_value);
      feedback->number_of_random_values_calculated++;
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class RandomizerActionServer

}  // namespace random_generator

RCLCPP_COMPONENTS_REGISTER_NODE(random_generator::RandomizerActionServer)