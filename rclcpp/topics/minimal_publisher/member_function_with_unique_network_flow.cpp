// Copyright 2020 Ericsson AB
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

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher_options.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisherWithUniqueNetworkFlow : public rclcpp::Node
{
public:
  MinimalPublisherWithUniqueNetworkFlow()
  : Node("minimal_publisher_with_unique_network_flow"), count_1_(0), count_2_(0)
  {
    // Enable unique network flow via options
    auto options_1 = rclcpp::PublisherOptions();
    options_1.require_unique_network_flow = RMW_UNIQUE_NETWORK_FLOW_STRICTLY_REQUIRED;
    publisher_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10, options_1);
    timer_1_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisherWithUniqueNetworkFlow::timer_1_callback, this));

    // Unique network flow is disabled by default
    auto options_2 = rclcpp::PublisherOptions();
    publisher_2_ = this->create_publisher<std_msgs::msg::String>("topic_2", 10);
    timer_2_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisherWithUniqueNetworkFlow::timer_2_callback, this));

    // Print network flows and check for uniqueness
    auto network_flows_1 = publisher_1_->get_network_flow();
    auto network_flows_2 = publisher_2_->get_network_flow();
    print_network_flows(network_flows_1);
    print_network_flows(network_flows_2);
    if (!are_network_flows_unique(network_flows_1, network_flows_2)) {
      RCLCPP_ERROR(this->get_logger(), "Network flows across publishers are not unique");
    }
  }

private:
  void timer_1_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_1_++);

    RCLCPP_INFO(
      this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_1_->publish(message);
  }
  void timer_2_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hej, världen! " + std::to_string(count_2_++);

    RCLCPP_INFO(
      this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_2_->publish(message);
  }
  /// Compare network flows
  /*
   * Compare two network flows
   * \return false if network flows are not unique true otherwise
   */
  bool are_network_flows_unique(
    const std::vector<rclcpp::NetworkFlow> & network_flows_1,
    const std::vector<rclcpp::NetworkFlow> & network_flows_2) const
  {
    if (network_flows_1.size() > 0 && network_flows_2.size() > 0) {
      for (auto network_flow_1 : network_flows_1) {
        for (auto network_flow_2 : network_flows_2) {
          if (network_flow_1 == network_flow_2) {
            return false;
          }
        }
      }
    }
    return true;
  }
  /// Print network flows
  void print_network_flows(const std::vector<rclcpp::NetworkFlow> & network_flows) const
  {
    std::ostringstream stream;
    for (auto network_flow : network_flows) {
      stream << network_flow << ", ";
    }
    RCLCPP_INFO(
      this->get_logger(), "Publisher created with network flows: '%s'",
      stream.str().c_str());
  }
  rclcpp::TimerBase::SharedPtr timer_1_;
  rclcpp::TimerBase::SharedPtr timer_2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_;
  size_t count_1_;
  size_t count_2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisherWithUniqueNetworkFlow>());
  rclcpp::shutdown();
  return 0;
}
