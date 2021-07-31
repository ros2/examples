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

#include <cassert>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("wait_set_example_node");

  auto do_nothing = [](std_msgs::msg::String::UniquePtr) {assert(false);};
  auto sub1 = node->create_subscription<std_msgs::msg::String>("~/chatter", 10, do_nothing);
  auto sub2 = node->create_subscription<std_msgs::msg::String>("~/chatter", 10, do_nothing);
  std::vector<decltype(sub1)> sub_vector = {sub1, sub2};
  auto guard_condition1 = std::make_shared<rclcpp::GuardCondition>();
  auto guard_condition2 = std::make_shared<rclcpp::GuardCondition>();

  // FIXME: removing sub if it was added in the ctor leads to a failure
  // terminate called after throwing an instance of 'std::runtime_error'
  //  what():  waitable not in wait set
  rclcpp::ThreadSafeWaitSet wait_set(
    // std::vector<rclcpp::WaitSet::SubscriptionEntry>{{sub}},
    {},
    std::vector<rclcpp::GuardCondition::SharedPtr>{guard_condition1});

  wait_set.add_subscription(sub1);  // FIXME: add it in the ctor
  wait_set.add_subscription(sub2);
  wait_set.add_guard_condition(guard_condition2);

  auto wait_and_print_results = [&]() {
      RCLCPP_INFO(node->get_logger(), "Waiting...");
      auto wait_result = wait_set.wait(std::chrono::seconds(1));
      if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
        size_t guard_conditions_num = wait_set.get_rcl_wait_set().size_of_guard_conditions;
        size_t subscriptions_num = wait_set.get_rcl_wait_set().size_of_subscriptions;

        for (size_t i = 0; i < guard_conditions_num; i++) {
          if (wait_result.get_wait_set().get_rcl_wait_set().guard_conditions[i]) {
            RCLCPP_INFO(node->get_logger(), "guard_condition %zu triggered", i + 1);
          }
        }
        for (size_t i = 0; i < subscriptions_num; i++) {
          if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[i]) {
            RCLCPP_INFO(node->get_logger(), "subscription %zu triggered", i + 1);
            std_msgs::msg::String msg;
            rclcpp::MessageInfo msg_info;
            if (sub_vector.at(i)->take(msg, msg_info)) {
              RCLCPP_INFO(
                node->get_logger(),
                "subscription %zu: I heard '%s'", i + 1, msg.data.c_str());
            } else {
              RCLCPP_INFO(node->get_logger(), "subscription %zu: No message", i + 1);
            }
          }
        }
      } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
        RCLCPP_INFO(node->get_logger(), "wait-set waiting failed with timeout");
      } else if (wait_result.kind() == rclcpp::WaitResultKind::Empty) {
        RCLCPP_INFO(node->get_logger(), "wait-set waiting failed because wait-set is empty");
      }
    };

  RCLCPP_INFO(node->get_logger(), "Action: Nothing triggered");
  wait_and_print_results();

  RCLCPP_INFO(node->get_logger(), "Action: Trigger Guard condition 1");
  guard_condition1->trigger();
  wait_and_print_results();

  RCLCPP_INFO(node->get_logger(), "Action: Trigger Guard condition 2");
  guard_condition2->trigger();
  wait_and_print_results();

  RCLCPP_INFO(node->get_logger(), "Action: Message published");
  auto pub = node->create_publisher<std_msgs::msg::String>("~/chatter", 1);
  pub->publish(std_msgs::msg::String().set__data("test"));
  wait_and_print_results();

  RCLCPP_INFO(node->get_logger(), "Action: Guard condition 1 removed");
  RCLCPP_INFO(node->get_logger(), "Action: Guard condition 2 removed");
  wait_set.remove_guard_condition(guard_condition1);
  wait_set.remove_guard_condition(guard_condition2);
  wait_and_print_results();

  RCLCPP_INFO(node->get_logger(), "Action: Subscription 2 removed");
  wait_set.remove_subscription(sub2);
  wait_and_print_results();

  RCLCPP_INFO(node->get_logger(), "Action: Subscription 1 removed");
  wait_set.remove_subscription(sub1);
  wait_and_print_results();


  return 0;
}
