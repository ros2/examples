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
#include <rclcpp/strategies/bounded_msg_memory_strategy.hpp>

#include <std_interfaces/msg/int32.hpp>
#include <std_interfaces/msg/string.hpp>

using namespace rclcpp::bounded_message_memory_strategy;

void chatterCallback(const std_interfaces::msg::Int32::ConstSharedPtr & msg)
{
  std::cout << "I heard: [" << msg->data << "]" << std::endl;
}

void chatterCallbackString(const std_interfaces::msg::String::ConstSharedPtr & msg)
{
  std::cout << "I heard: [" << msg->data << "]" << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("listener_bounded");

  BoundedMessageMemoryStrategy<std_interfaces::msg::Int32> msg_strategy;
  auto msg_strategy_ptr = std::make_shared<message_memory_strategy::MessageMemoryStrategy<std_interfaces::msg::Int32>>(msg_strategy);
  auto sub = node->create_subscription<std_interfaces::msg::Int32>("chatter", 7, chatterCallback, nullptr, false, msg_strategy_ptr);

  // If the following block is uncommented and the above block commented out, then the example will not compile.
  /*
  BoundedMessageMemoryStrategy<std_interfaces::msg::String> msg_strategy;
  auto msg_strategy_ptr = std::make_shared<message_memory_strategy::MessageMemoryStrategy<std_interfaces::msg::String>>(msg_strategy);
  auto sub = node->create_subscription<std_interfaces::msg::String>("chatter", 7, chatterCallbackString, nullptr, false, msg_strategy_ptr);
  */

  rclcpp::spin(node);

  return 0;
}
