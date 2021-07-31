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

/* For this example, we will be creating a talker node with three publishers which will
 * publish the topics A, B, C in random order each time. In this case the messages are handled
 * using an executor. The order in which the messages are handled will depend on the message
 * arrival and the type of messages available at the moment.
 */

int32_t main(const int32_t argc, char ** const argv)
{
  rclcpp::init(argc, argv);

  // Note the order of execution would be deterministic if both nodes are spun in the same
  // executor (A, B, C). This is because the publishing happens always before the subscription
  // handling and the executor handles the messages in the order in which the subscriptions were
  // created. Using different threads the handling order depends on the message arrival and
  // type of messages available.
  auto thread = std::thread([]() {rclcpp::spin(std::make_shared<RandomTalker>());});
  rclcpp::spin(std::make_shared<RandomListener>());

  rclcpp::shutdown();
  thread.join();

  return 0;
}
