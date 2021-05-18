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

class MinimalPublisherWithUniqueNetworkFlowEndpoints : public rclcpp::Node
{
public:
  MinimalPublisherWithUniqueNetworkFlowEndpoints()
  : Node("minimal_publisher_with_unique_network_flow_endpoints"), count_1_(0), count_2_(0)
  {
    // Create publisher with unique network flow endpoints
    // Enable unique network flow endpoints via options
    auto options_1 = rclcpp::PublisherOptions();
    options_1.require_unique_network_flow_endpoints =
      RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED;
    publisher_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10, options_1);
    timer_1_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisherWithUniqueNetworkFlowEndpoints::timer_1_callback, this));

    // Create publisher without unique network flow endpoints
    // Unique network flow endpoints are disabled in default options
    publisher_2_ = this->create_publisher<std_msgs::msg::String>("topic_2", 10);
    timer_2_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisherWithUniqueNetworkFlowEndpoints::timer_2_callback, this));

    // Catch an exception if implementation does not support get_network_flow_endpoints.
    try {
      // Get network flow endpoints
      auto network_flow_endpoints_1 = publisher_1_->get_network_flow_endpoints();
      auto network_flow_endpoints_2 = publisher_2_->get_network_flow_endpoints();

      // Print network flow endpoints
      print_network_flow_endpoints(network_flow_endpoints_1);
      print_network_flow_endpoints(network_flow_endpoints_2);
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_INFO(
        this->get_logger(), "%s", e.what());
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
  /// Print network flow endpoints in JSON-like format
  void print_network_flow_endpoints(
    const std::vector<rclcpp::NetworkFlowEndpoint> & network_flow_endpoints) const
  {
    std::ostringstream stream;
    stream << "{\"networkFlowEndpoints\": [";
    bool comma_skip = true;
    for (auto network_flow_endpoint : network_flow_endpoints) {
      if (comma_skip) {
        comma_skip = false;
      } else {
        stream << ",";
      }
      stream << network_flow_endpoint;
    }
    stream << "]}";
    RCLCPP_INFO(
      this->get_logger(), "%s",
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
  rclcpp::spin(std::make_shared<MinimalPublisherWithUniqueNetworkFlowEndpoints>());
  rclcpp::shutdown();
  return 0;
}
