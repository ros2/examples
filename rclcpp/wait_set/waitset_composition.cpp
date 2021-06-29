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
    subscription_executor_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s' (executor)", msg->data.c_str());
      });

    // creates a subscription which is not automatically added to an executor
    rclcpp::CallbackGroup::SharedPtr cb_group_waitset = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = cb_group_waitset;
    auto do_nothing = [](std_msgs::msg::String::UniquePtr) {assert(false);};
    subscription_waitset_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      do_nothing,
      options);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr get_subscription() const
  {
    return subscription_waitset_;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_waitset_;
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

  // Create a static wait-set that will wait on the subscription that will excluded from the
  // default executor callback group
  rclcpp::StaticWaitSet<1, 0, 0, 0, 0, 0> wait_set({{{listener->get_subscription()}}});

  auto thread = std::thread(
    [&exec]() {
      // spin will block until work comes in, execute work as it becomes available, and keep blocking.
      // It will only be interrupted by Ctrl-C.
      exec.spin();
    });

  while (rclcpp::ok()) {
    // Waiting up to 5s for a message to arrive
    const auto wait_result = wait_set.wait(std::chrono::seconds(5));

    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U]) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        if (listener->get_subscription()->take(msg, msg_info)) {
          RCLCPP_INFO(listener->get_logger(), "I heard: '%s' (waitset)", msg.data.c_str());
        }
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      RCLCPP_INFO(listener->get_logger(), "No message received after 5s.");
    } else {
      RCLCPP_INFO(listener->get_logger(), "Wait-set failed.");
    }
  }

  rclcpp::shutdown();
  thread.join();
  return 0;
}
