// Copyright (c) 2020 Robert Bosch GmbH
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

#ifndef EXAMPLES_RCLCPP_CBG_EXECUTOR__PONG_NODE_HPP_
#define EXAMPLES_RCLCPP_CBG_EXECUTOR__PONG_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace examples_rclcpp_cbg_executor
{

class PongNode : public rclcpp::Node
{
public:
  PongNode();

  virtual ~PongNode() = default;

  rclcpp::CallbackGroup::SharedPtr get_high_prio_callback_group();

  rclcpp::CallbackGroup::SharedPtr get_low_prio_callback_group();

private:
  rclcpp::CallbackGroup::SharedPtr low_prio_callback_group_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr high_ping_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr high_pong_publisher_;
  void high_ping_received(const std_msgs::msg::Int32::ConstSharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr low_ping_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr low_pong_publisher_;
  void low_ping_received(const std_msgs::msg::Int32::ConstSharedPtr msg);

  static void burn_cpu_cycles(std::chrono::nanoseconds duration);
};

}  // namespace examples_rclcpp_cbg_executor

#endif  // EXAMPLES_RCLCPP_CBG_EXECUTOR__PONG_NODE_HPP_
