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
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "random_generator/action/randomizer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace random_generator {
class RandomizerActionClient : public rclcpp::Node {
 public:
  using Randomizer = action::Randomizer;
  using GoalHandleRandomizer = rclcpp_action::ClientGoalHandle<Randomizer>;

  explicit RandomizerActionClient(const rclcpp::NodeOptions& options)
      : Node("randomizer_action_client", options) {
    this->client_ptr_ = rclcpp_action::create_client<Randomizer>(this, "randomizer");

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                           std::bind(&RandomizerActionClient::send_goal, this));
  }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Randomizer::Goal();
    goal_msg.number_of_values = 10;
    goal_msg.min_val = 0.5;
    goal_msg.max_val = 9.5;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Randomizer>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&RandomizerActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&RandomizerActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&RandomizerActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

 private:
  rclcpp_action::Client<Randomizer>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleRandomizer::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleRandomizer::SharedPtr,
                         const std::shared_ptr<const Randomizer::Feedback> feedback) {
    std::stringstream ss;
    ss << std::setprecision(6);
    ss << "Next random number: " << feedback->new_random_value
       << ". \t Done: " << feedback->number_of_random_values_calculated << " values.";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleRandomizer::WrappedResult& result_handle) {
    switch (result_handle.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result_handle.result->random_values) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class RandomizerActionClient

}  // namespace random_generator

RCLCPP_COMPONENTS_REGISTER_NODE(random_generator::RandomizerActionClient)