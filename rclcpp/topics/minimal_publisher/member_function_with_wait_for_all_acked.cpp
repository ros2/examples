// Copyright 2021 Open Source Robotics Foundation, Inc.
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
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example shows how to use wait_for_all_acked for the publisher */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher_with_wait_for_all_acked"), count_(0), timeout_(500)
  {
    // publisher must set reliable mode
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "topic",
      rclcpp::QoS(10).reliable());
    timer_ = this->create_wall_timer(
      1s, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    // If no subscription is connected, wait_for_all_acked() always return true.
    if (publisher_->get_subscription_count() != 0) {
      if (publisher_->wait_for_all_acked(timeout_)) {
        RCLCPP_INFO(
          this->get_logger(),
          "Subscriber acknowledges message '%s'",
          message.data.c_str());
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "Subscriber doesn't acknowledge message '%s' during %ld ms",
          message.data.c_str(),
          timeout_.count());
      }
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std::chrono::milliseconds timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
