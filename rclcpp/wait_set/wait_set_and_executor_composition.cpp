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

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

class Listener : public rclcpp::Node
{
public:
  Listener()
  : Node("listener")
  {
    auto subscription_callback = [this](std_msgs::msg::String::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    };
    sub1_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      subscription_callback
      );

    rclcpp::CallbackGroup::SharedPtr cb_group_waitset = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = cb_group_waitset;
    sub2_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      subscription_callback,
      options);
    wait_set_.add_subscription(sub2_);
    thread_ = std::thread([this]() -> void {spin_wait_set();});
  }

  void spin_wait_set()
  {
    while (rclcpp::ok()) {
      // Waiting up to 5s for a message to arrive
      const auto wait_result = wait_set_.wait(std::chrono::seconds(5));
      if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
        if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U]) {
          std_msgs::msg::String msg;
          rclcpp::MessageInfo msg_info;
          if (sub2_->take(msg, msg_info)) {
            std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg);
            sub2_->handle_message(type_erased_msg, msg_info);
          }
        }
      } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Wait-set failed with timeout");
        }
      }
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
  rclcpp::WaitSet wait_set_;
  std::thread thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create an executor that will be responsible for execution of callbacks for a set of nodes.
  // All callbacks will be called except the subscription which was explicitly excluded from it
  rclcpp::executors::SingleThreadedExecutor exec;

  auto talker = std::make_shared<Talker>();
  exec.add_node(talker);
  auto listener = std::make_shared<Listener>();
  exec.add_node(listener);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
