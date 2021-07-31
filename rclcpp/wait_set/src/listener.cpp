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

#include <thread>
#include <memory>

#include "wait_set/listener.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

Listener::Listener(rclcpp::NodeOptions options)
: Node("listener", options)
{
  auto subscription_callback = [this](std_msgs::msg::String::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s' (executor)", msg->data.c_str());
    };
  subscription1_ = this->create_subscription<std_msgs::msg::String>(
    "topic",
    10,
    subscription_callback
  );

  auto wait_set_subscription_callback = [this](std_msgs::msg::String::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s' (wait-set)", msg->data.c_str());
    };
  rclcpp::CallbackGroup::SharedPtr cb_group_waitset = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto subscription_options = rclcpp::SubscriptionOptions();
  subscription_options.callback_group = cb_group_waitset;
  subscription2_ = this->create_subscription<std_msgs::msg::String>(
    "topic",
    10,
    wait_set_subscription_callback,
    subscription_options);
  wait_set_.add_subscription(subscription2_);
  thread_ = std::thread([this]() -> void {spin_wait_set();});
}

Listener::~Listener()
{
  if (thread_.joinable()) {
    thread_.join();
  }
}

void Listener::spin_wait_set()
{
  while (rclcpp::ok()) {
    // Waiting up to 1s for a message to arrive
    const auto wait_result = wait_set_.wait(std::chrono::seconds(1));
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U]) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        if (subscription2_->take(msg, msg_info)) {
          std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg);
          subscription2_->handle_message(type_erased_msg, msg_info);
        }
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Wait-set failed with timeout");
      }
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(Listener)
