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

class CustomContext : public rclcpp::Context
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(CustomContext)

  bool shutdown(const std::string & reason) override
  {
    // Before shutdown, confirm all subscribers acknowledge sent messages
    if (custom_shutdown_func_) {
      custom_shutdown_func_();
    }

    return rclcpp::Context::shutdown(reason);
  }

  void set_custom_shutdown_func(std::function<void()> custom_shutdown_func)
  {
    custom_shutdown_func_ = custom_shutdown_func;
  }

private:
  std::function<void()> custom_shutdown_func_;
};

class MinimalPublisher : public rclcpp::Node
{
public:
  explicit MinimalPublisher(const CustomContext::SharedPtr context)
  : Node("minimal_publisher_with_wait_for_all_acked", rclcpp::NodeOptions().context(context)),
    count_(0),
    wait_timeout_(300)
  {
    // publisher must set reliable mode
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "topic",
      rclcpp::QoS(10).reliable());

    context->set_custom_shutdown_func(
      [this]() {
        this->wait_for_all_acked();
      });

    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void wait_for_all_acked()
  {
    // Confirm all subscribers receive sent messages.
    // Note that if no subscription is connected, wait_for_all_acked() always return true.
    if (publisher_->wait_for_all_acked(wait_timeout_)) {
      RCLCPP_INFO(
        this->get_logger(),
        "All Subscribers acknowledges messages");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Not all subscribers acknowledge messages during %ld ms",
        wait_timeout_.count());
    }
  }

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    // After sending some messages, you can call wait_for_all_acked() to confirm all subscribers
    // acknowledge message.
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std::chrono::milliseconds wait_timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto custom_context = std::make_shared<CustomContext>();
  custom_context->init(argc, argv);

  auto publisher = std::make_shared<MinimalPublisher>(custom_context);
  rclcpp::spin(publisher);
  rclcpp::shutdown();

  return 0;
}
