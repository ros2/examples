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

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr get_subscription() const
  {
    return subscription_;
  }

  void run()
  {
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(subscription_);
    while (rclcpp::ok()) {
      const auto wait_result = wait_set.wait();
      if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
        if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U]) {
          std_msgs::msg::String msg;
          rclcpp::MessageInfo msg_info;
          bool message_received = subscription_->take(msg, msg_info);
          if (message_received) {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
          }
        }
      }
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();

  // Option 1: loop in main
  auto sub = node->get_subscription();
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(sub);
  while (rclcpp::ok()) {
    const auto wait_result = wait_set.wait();
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U]) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        bool message_received = sub->take(msg, msg_info);
        if (message_received) {
          RCLCPP_INFO(node->get_logger(), "I heard: '%s'", msg.data.c_str());
        }
      }
    }
  }

  // Option 2: run node
  // node->run();

  // rclcpp::spin();
  rclcpp::shutdown();
  return 0;
}
