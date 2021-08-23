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

#ifndef EXAMPLES_RCLCPP_CBG_EXECUTOR__PING_NODE_HPP_
#define EXAMPLES_RCLCPP_CBG_EXECUTOR__PING_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace examples_rclcpp_cbg_executor
{

struct RTTData
{
  explicit RTTData(const rclcpp::Time & sent)
  : sent_(sent) {}
  rclcpp::Time sent_{0, 0};
  rclcpp::Time high_received_{0, 0};
  rclcpp::Time low_received_{0, 0};
};

class PingNode : public rclcpp::Node
{
public:
  PingNode();

  virtual ~PingNode() = default;

  void print_statistics(std::chrono::seconds experiment_duration) const;

private:
  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr high_ping_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr low_ping_publisher_;
  void send_ping();

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr high_pong_subscription_;
  void high_pong_received(const std_msgs::msg::Int32::ConstSharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr low_pong_subscription_;
  void low_pong_received(const std_msgs::msg::Int32::ConstSharedPtr msg);

  std::vector<RTTData> rtt_data_;
};

}  // namespace examples_rclcpp_cbg_executor

#endif  // EXAMPLES_RCLCPP_CBG_EXECUTOR__PING_NODE_HPP_
