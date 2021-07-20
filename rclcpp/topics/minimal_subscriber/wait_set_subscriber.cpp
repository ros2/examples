// Copyright 2021, Apex.AI Inc.
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
#include "std_msgs/msg/string.hpp"

/* This example creates a subclass of Node and uses a wait-set based loop to wait on
 * a subscription to have messages available and then handles them manually without an executor */

class WaitSetSubscriber : public rclcpp::Node
{
public:
  explicit WaitSetSubscriber(rclcpp::NodeOptions options)
  : Node("wait_set_subscriber", options)
  {
    rclcpp::CallbackGroup::SharedPtr cb_group_waitset = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_waitset;
    auto subscription_callback = [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      subscription_callback,
      subscription_options);
    wait_set_.add_subscription(subscription_);
    thread_ = std::thread([this]() -> void {spin_wait_set();});
  }

  ~WaitSetSubscriber()
  {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void spin_wait_set()
  {
    while (rclcpp::ok()) {
      // Wait for the subscriber event to trigger. Set a 1 ms margin to trigger a timeout.
      const auto wait_result = wait_set_.wait(std::chrono::milliseconds(501));
      switch (wait_result.kind()) {
        case rclcpp::WaitResultKind::Ready:
          {
            std_msgs::msg::String msg;
            rclcpp::MessageInfo msg_info;
            if (subscription_->take(msg, msg_info)) {
              std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg);
              subscription_->handle_message(type_erased_msg, msg_info);
            }
            break;
          }
        case rclcpp::WaitResultKind::Timeout:
          if (rclcpp::ok()) {
            RCLCPP_WARN(this->get_logger(), "Timeout. No message received after given wait-time");
          }
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Error. Wait-set failed.");
      }
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::WaitSet wait_set_;
  std::thread thread_;
};

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(WaitSetSubscriber)
