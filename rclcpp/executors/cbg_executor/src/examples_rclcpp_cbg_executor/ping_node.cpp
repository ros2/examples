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

#include "examples_rclcpp_cbg_executor/ping_node.hpp"

#include <algorithm>
#include <functional>
#include <memory>

#include "./utilities.hpp"

namespace examples_rclcpp_cbg_executor
{

PingNode::PingNode()
: rclcpp::Node("ping_node")
{
  using std::placeholders::_1;
  using std_msgs::msg::Int32;

  this->declare_parameter<double>("ping_period", 0.01);
  std::chrono::nanoseconds ping_period = get_nanos_from_secs_parameter(this, "ping_period");

  ping_timer_ = this->create_wall_timer(ping_period, std::bind(&PingNode::send_ping, this));
  high_ping_publisher_ = this->create_publisher<Int32>("high_ping", rclcpp::SensorDataQoS());
  low_ping_publisher_ = this->create_publisher<Int32>("low_ping", rclcpp::SensorDataQoS());

  high_pong_subscription_ = this->create_subscription<Int32>(
    "high_pong", rclcpp::SensorDataQoS(), std::bind(&PingNode::high_pong_received, this, _1));
  low_pong_subscription_ = this->create_subscription<Int32>(
    "low_pong", rclcpp::SensorDataQoS(), std::bind(&PingNode::low_pong_received, this, _1));
}

void PingNode::send_ping()
{
  std_msgs::msg::Int32 msg;
  msg.data = static_cast<int32_t>(rtt_data_.size());
  rtt_data_.push_back(RTTData(now()));
  high_ping_publisher_->publish(msg);
  low_ping_publisher_->publish(msg);
}

void PingNode::high_pong_received(const std_msgs::msg::Int32::SharedPtr msg)
{
  rtt_data_[msg->data].high_received_ = now();
}

void PingNode::low_pong_received(const std_msgs::msg::Int32::SharedPtr msg)
{
  rtt_data_[msg->data].low_received_ = now();
}

void PingNode::print_statistics(std::chrono::seconds experiment_duration) const
{
  size_t ping_count = rtt_data_.size();

  size_t high_pong_count = 0;
  size_t low_pong_count = 0;
  rclcpp::Duration high_rtt_sum(0, 0);
  rclcpp::Duration low_rtt_sum(0, 0);
  for (const auto & entry : rtt_data_) {
    if (entry.high_received_.nanoseconds() >= entry.sent_.nanoseconds()) {
      ++high_pong_count;
      high_rtt_sum = high_rtt_sum + (entry.high_received_ - entry.sent_);
    }
    if (entry.low_received_.nanoseconds() >= entry.sent_.nanoseconds()) {
      ++low_pong_count;
      low_rtt_sum = low_rtt_sum + (entry.low_received_ - entry.sent_);
    }
  }

  std::chrono::nanoseconds ping_period = get_nanos_from_secs_parameter(this, "ping_period");
  size_t ideal_ping_count = experiment_duration / ping_period;
  RCLCPP_INFO(
    get_logger(), "Both paths: Sent out %zu of configured %ld pings, i.e. %zu%%.",
    ping_count, ideal_ping_count, 100 * ping_count / ideal_ping_count);
  RCLCPP_INFO(
    get_logger(), "High prio path: Received %zu pongs, i.e. for %zu%% of the pings.",
    high_pong_count, 100 * high_pong_count / ping_count);
  if (high_pong_count > 0) {
    double high_rtt_avg = (high_rtt_sum.seconds() * 1000.0 / high_pong_count);
    RCLCPP_INFO(get_logger(), "High prio path: Average RTT is %3.1fms.", high_rtt_avg);
  }

  RCLCPP_INFO(
    get_logger(), "Low prio path: Received %zu pongs, i.e. for %zu%% of the pings.",
    low_pong_count, 100 * low_pong_count / ping_count);
  if (low_pong_count > 0) {
    double low_rtt_avg = (low_rtt_sum.seconds() * 1000.0 / low_pong_count);
    RCLCPP_INFO(get_logger(), "Low prio path: Average RTT is %3.1fms.", low_rtt_avg);
  }
}

}  // namespace examples_rclcpp_cbg_executor
