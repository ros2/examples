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
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/memory_strategies.hpp>

#include <example_interfaces/msg/large_fixed.hpp>

using namespace rclcpp::memory_strategies::stack_pool_memory_strategy;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy = nullptr;
  if (argc > 1) {
    std::string argument(argv[1]);
    if (argument == "stack_pool") {
      printf("Setting memory allocation strategy to 'stack_pool'.\n");
      memory_strategy = std::make_shared<StackPoolMemoryStrategy<>>();
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

  auto node = rclcpp::node::Node::make_shared("talker");

  rclcpp::executors::SingleThreadedExecutor executor(memory_strategy);

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  executor.add_node(node);

  auto chatter_pub = node->create_publisher<example_interfaces::msg::LargeFixed>("chatter",
      custom_qos_profile);

  auto msg = std::make_shared<example_interfaces::msg::LargeFixed>();
  auto i = 1;
  for (size_t j = 0; j < 255; ++j) {
    msg->data[j] = i + '0';
  }
  ++i;

  auto publish_callback =
    [&chatter_pub, &msg, &i]()
    {
      std::cout << "publishing" << std::endl;
      chatter_pub->publish(msg);
      for (size_t j = 0; j < 255; ++j) {
        msg->data[j] = i;
      }
      ++i;
    };

  auto timer = node->create_wall_timer(std::chrono::nanoseconds(1000000000), publish_callback);

  executor.spin();

  return 0;
}
