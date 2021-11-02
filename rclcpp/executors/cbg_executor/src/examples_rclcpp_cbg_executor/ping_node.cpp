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
#include <vector>

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

void PingNode::high_pong_received(const std_msgs::msg::Int32::ConstSharedPtr msg)
{
  rtt_data_[msg->data].high_received_ = now();
}

void PingNode::low_pong_received(const std_msgs::msg::Int32::ConstSharedPtr msg)
{
  rtt_data_[msg->data].low_received_ = now();
}

void PingNode::print_statistics(std::chrono::seconds experiment_duration) const
{
  size_t ping_count = rtt_data_.size();

  std::vector<double> high_rtts;
  std::vector<double> low_rtts;

  for (const auto & entry : rtt_data_) {
    if (entry.high_received_.nanoseconds() >= entry.sent_.nanoseconds()) {
      high_rtts.push_back((entry.high_received_ - entry.sent_).seconds());
    }
    if (entry.low_received_.nanoseconds() >= entry.sent_.nanoseconds()) {
      low_rtts.push_back((entry.low_received_ - entry.sent_).seconds());
    }
  }

  std::chrono::nanoseconds ping_period = get_nanos_from_secs_parameter(this, "ping_period");
  size_t ideal_ping_count = experiment_duration / ping_period;
  RCLCPP_INFO(
    get_logger(), "Both paths: Sent out %zu of configured %ld pings, i.e. %zu%%.",
    ping_count, ideal_ping_count, 100 * ping_count / ideal_ping_count);
  RCLCPP_INFO(
    get_logger(), "High prio path: Received %zu pongs, i.e. for %zu%% of the pings.",
    high_rtts.size(), 100 * high_rtts.size() / ping_count);
  if (!high_rtts.empty()) {
    double high_rtt_avg = calc_average(high_rtts) * 1000.0;
    RCLCPP_INFO(get_logger(), "High prio path: Average RTT is %3.1fms.", high_rtt_avg);
    double high_rtt_jitter = calc_std_deviation(high_rtts) * 1000.0;
    RCLCPP_INFO(get_logger(), "High prio path: Jitter of RTT is %5.3fms.", high_rtt_jitter);
  }

  RCLCPP_INFO(
    get_logger(), "Low prio path: Received %zu pongs, i.e. for %zu%% of the pings.",
    low_rtts.size(), 100 * low_rtts.size() / ping_count);
  if (!low_rtts.empty()) {
    double low_rtt_avg = calc_average(low_rtts) * 1000.0;
    RCLCPP_INFO(get_logger(), "Low prio path: Average RTT is %3.1fms.", low_rtt_avg);
    double low_rtt_jitter = calc_std_deviation(low_rtts) * 1000.0;
    RCLCPP_INFO(get_logger(), "Low prio path: Jitter of RTT is %5.3fms.", low_rtt_jitter);
  }
}

}  // namespace examples_rclcpp_cbg_executor
