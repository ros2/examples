// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/memory_strategies.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>

#include <example_interfaces/msg/large_fixed.hpp>

using namespace rclcpp::strategies::message_pool_memory_strategy;
using namespace rclcpp::memory_strategies;

size_t messages_received = 0;

void chatterCallback(const example_interfaces::msg::LargeFixed::SharedPtr msg)
{
  printf("Address of received message: %p\n", msg.get());
  ++messages_received;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy = nullptr;
  if (argc > 1) {
    std::string argument(argv[1]);

    if (argument == "stack_pool") {
      printf("Setting memory allocation strategy to 'stack_pool'.\n");
      memory_strategy = std::make_shared<StackPoolMemoryStrategy<1, 1, 1, 1, 2, 0>>();
    } else if (argument == "realtime") {
      printf("Setting memory allocation strategy to 'heap_pool'.\n");
      rclcpp::memory_strategies::heap_pool_memory_strategy::ObjectPoolBounds bounds;
      bounds.set_max_subscriptions(1).set_max_services(1).set_max_clients(1);
      bounds.set_max_executables(1).set_memory_pool_size(0);
      memory_strategy = std::make_shared<HeapPoolMemoryStrategy>(bounds);
    } else if (argument == "dynamic") {
      printf("Setting memory allocation strategy to 'dynamic'.\n");
    } else {
      fprintf(stderr, "Warning: unknown argument.\n");
      printf("Setting memory allocation strategy to default (dynamic).\n");
    }
  }
  if (!memory_strategy) {
    memory_strategy = rclcpp::memory_strategy::create_default_strategy();
  }

  rclcpp::executors::SingleThreadedExecutor executor(memory_strategy);
  auto node = rclcpp::Node::make_shared("listener_single_msg");

  executor.add_node(node);

  auto msg_strategy_ptr =
    std::make_shared<MessagePoolMemoryStrategy<example_interfaces::msg::LargeFixed, 1>>();

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  auto sub = node->create_subscription<example_interfaces::msg::LargeFixed>(
    "chatter", custom_qos_profile, chatterCallback, nullptr, false,
    msg_strategy_ptr);

  executor.spin();

  printf("Received %zu messages.\n", messages_received);

  return 0;
}
