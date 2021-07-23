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

#include "wait_set/random_listener.hpp"
#include "wait_set/random_talker.hpp"

using namespace std::chrono_literals;

/* For this example, we will be creating a talker node with three publishers which will
 * publish the topics A, B, C in random order each time. The order in which the messages are
 * handled is defined deterministically by the user in the code using a wait-set based loop.
 * That is, in this example we always take and process the data in the same order  A, B, C
 * regardless of the arrival order. */

int32_t main(const int32_t argc, char ** const argv)
{
  rclcpp::init(argc, argv);

  auto random_listener = std::make_shared<RandomListener>();
  auto subscriptions = random_listener->get_subscriptions();

  // Create a wait-set and add the subscriptions
  rclcpp::WaitSet wait_set;
  for (const auto & subscription : subscriptions) {
    wait_set.add_subscription(subscription);
  }

  // Create a random talker and start publishing in another thread
  auto thread = std::thread([]() {rclcpp::spin(std::make_shared<RandomTalker>());});

  while (rclcpp::ok()) {
    const auto wait_result = wait_set.wait(2s);
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      bool sub1_has_data = wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U];
      bool sub2_has_data = wait_result.get_wait_set().get_rcl_wait_set().subscriptions[1U];
      bool sub3_has_data = wait_result.get_wait_set().get_rcl_wait_set().subscriptions[2U];

      // Handle all the messages when all subscriptions have data
      if (sub1_has_data && sub2_has_data && sub3_has_data) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        const size_t subscriptions_num = wait_set.get_rcl_wait_set().size_of_subscriptions;
        for (size_t i = 0; i < subscriptions_num; i++) {
          if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[i]) {
            if (subscriptions.at(i)->take(msg, msg_info)) {
              std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg);
              subscriptions.at(i)->handle_message(type_erased_msg, msg_info);
            }
          }
        }
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(random_listener->get_logger(), "Wait-set failed with timeout");
      }
    }
  }

  rclcpp::shutdown();
  thread.join();
  return 0;
}
