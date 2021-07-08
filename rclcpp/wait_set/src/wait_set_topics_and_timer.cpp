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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a node with three publishers, three subscriptions and a one-off timer. The
 * node will use a wait-set based loop to trigger the timer, publish the messages and handle the
 * received data */

int32_t main(const int32_t argc, char ** const argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("wait_set_listener");
  auto do_nothing = [](std_msgs::msg::String::UniquePtr) {assert(false);};

  auto sub1 = node->create_subscription<std_msgs::msg::String>("topicA", 1, do_nothing);
  auto sub2 = node->create_subscription<std_msgs::msg::String>("topicB", 1, do_nothing);
  auto sub3 = node->create_subscription<std_msgs::msg::String>("topicC", 1, do_nothing);

  std_msgs::msg::String msg1, msg2, msg3;
  msg1.data = "Hello, world!";
  msg2.data = "Hello, world!";
  msg3.data = "Hello, world!";

  const auto pub1 =
    node->create_publisher<std_msgs::msg::String>("topicA", 1);
  const auto pub2 =
    node->create_publisher<std_msgs::msg::String>("topicB", 1);
  const auto pub3 =
    node->create_publisher<std_msgs::msg::String>("topicC", 1);

  // Use a timer to schedule one-off message publishing.
  // Note in this case the callback won't be triggered automatically. It is up to the user to
  // trigger it manually inside the wait-set loop.
  rclcpp::TimerBase::SharedPtr one_off_timer;
  auto timer_callback = [&]() {
      RCLCPP_INFO(node->get_logger(), "Publishing msg1: '%s'", msg1.data.c_str());
      RCLCPP_INFO(node->get_logger(), "Publishing msg2: '%s'", msg2.data.c_str());
      RCLCPP_INFO(node->get_logger(), "Publishing msg3: '%s'", msg3.data.c_str());
      pub1->publish(msg1);
      pub2->publish(msg2);
      pub3->publish(msg3);
      // disable the timer after the first call
      one_off_timer->cancel();
    };

  one_off_timer = node->create_wall_timer(1s, timer_callback);

  rclcpp::WaitSet wait_set({{{sub1}, {sub2}, {sub3}}}, {}, {one_off_timer});

  // Loop waiting on the wait-set until 3 messages are received
  auto num_recv = std::size_t();
  while (num_recv < 3U) {
    const auto wait_result = wait_set.wait(2s);
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      if (wait_result.get_wait_set().get_rcl_wait_set().timers[0U]) {
        // The timer callback is executed manually here
        one_off_timer->execute_callback();
      } else {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U]) {
          if (sub1->take(msg, msg_info)) {
            ++num_recv;
            RCLCPP_INFO(node->get_logger(), "msg1 data: '%s'", msg.data.c_str());
          }
        }
        if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[1U]) {
          if (sub2->take(msg, msg_info)) {
            ++num_recv;
            RCLCPP_INFO(node->get_logger(), "msg2 data: '%s'", msg.data.c_str());
          }
        }
        if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[2U]) {
          if (sub3->take(msg, msg_info)) {
            ++num_recv;
            RCLCPP_INFO(node->get_logger(), "msg3 data: '%s'", msg.data.c_str());
          }
        }
        RCLCPP_INFO(node->get_logger(), "Number of messages already got: %zu of 3", num_recv);
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Wait-set failed with timeout");
      }
    }
  }
  RCLCPP_INFO(node->get_logger(), "Got all messages!");

  return 0;
}
