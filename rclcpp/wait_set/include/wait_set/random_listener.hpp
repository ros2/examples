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

#ifndef WAIT_SET__RANDOM_LISTENER_HPP_
#define WAIT_SET__RANDOM_LISTENER_HPP_

#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RandomListener : public rclcpp::Node
{
  using subscription_list = std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>;

public:
  RandomListener()
  : Node("random_listener")
  {
    auto print_msg = [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    sub1_ = this->create_subscription<std_msgs::msg::String>("topicA", 10, print_msg);
    sub2_ = this->create_subscription<std_msgs::msg::String>("topicB", 10, print_msg);
    sub3_ = this->create_subscription<std_msgs::msg::String>("topicC", 10, print_msg);
  }

  subscription_list get_subscriptions() const
  {
    return {sub1_, sub2_, sub3_};
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub3_;
};
#endif  // WAIT_SET__RANDOM_LISTENER_HPP_
