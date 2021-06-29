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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <random>

using namespace std::chrono_literals;

class RandomTalker : public rclcpp::Node
{
public:
  RandomTalker()
  : Node("random_talker"),
    pub1_(this->create_publisher<std_msgs::msg::String>("topicA", 10)),
    pub2_(this->create_publisher<std_msgs::msg::String>("topicB", 10)),
    pub3_(this->create_publisher<std_msgs::msg::String>("topicC", 10)),
    rand_engine_(std::chrono::system_clock::now().time_since_epoch().count())
  {
    publish_functions_.emplace_back(
      ([this]() {
        std_msgs::msg::String msg;
        msg.data = "A";
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
        pub1_->publish(msg);
      }));
    publish_functions_.emplace_back(
      ([this]() {
        std_msgs::msg::String msg;
        msg.data = "B";
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
        pub2_->publish(msg);
      }));
    publish_functions_.emplace_back(
      ([this]() {
        std_msgs::msg::String msg;
        msg.data = "C";
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
        pub3_->publish(msg);
      }));
    auto timer_callback =
      [this]() -> void {
        std::shuffle(publish_functions_.begin(), publish_functions_.end(), rand_engine_);
        for (const auto & f : publish_functions_) {f();}
      };
    timer_ = this->create_wall_timer(1s, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub3_;
  std::vector<std::function<void()>> publish_functions_;
  std::default_random_engine rand_engine_;
};
