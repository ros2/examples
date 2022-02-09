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


#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriberWithUniqueNetworkFlowEndpoints : public rclcpp::Node
{
public:
  MinimalSubscriberWithUniqueNetworkFlowEndpoints()
  : Node("minimal_subscriber_with_unique_network_flow_endpoints")
  {
    try {
      // Create subscription with unique network flow endpoints
      // Enable unique network flow endpoints via options
      // Since option is strict, expect exception
      auto options_1 = rclcpp::SubscriptionOptions();
      options_1.require_unique_network_flow_endpoints =
        RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED;

      subscription_1_ = this->create_subscription<std_msgs::msg::String>(
        "topic_1", 10, std::bind(
          &MinimalSubscriberWithUniqueNetworkFlowEndpoints::topic_1_callback, this,
          _1), options_1);

      // Create subscription without unique network flow endpoints
      // Unique network flow endpoints are disabled by default
      auto options_2 = rclcpp::SubscriptionOptions();
      subscription_2_ = this->create_subscription<std_msgs::msg::String>(
        "topic_2", 10, std::bind(
          &MinimalSubscriberWithUniqueNetworkFlowEndpoints::topic_2_callback, this,
          _1), options_2);

      // Get network flow endpoints
      auto network_flow_endpoints_1 = subscription_1_->get_network_flow_endpoints();
      auto network_flow_endpoints_2 = subscription_2_->get_network_flow_endpoints();

      // Check if network flow endpoints are unique
      for (auto network_flow_endpoint_1 : network_flow_endpoints_1) {
        for (auto network_flow_endpoint_2 : network_flow_endpoints_2) {
          if (network_flow_endpoint_1 == network_flow_endpoint_2) {
            RCLCPP_ERROR(
              this->get_logger(), "Network flow endpoints across subscriptions are not unique");
            break;
          }
        }
      }

      // Print network flow endpoints
      print_network_flow_endpoints(network_flow_endpoints_1);
      print_network_flow_endpoints(network_flow_endpoints_2);
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Error: %s",
        e.what());
    }
  }

private:
  void topic_1_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Topic 1 news: '%s'", msg.data.c_str());
  }
  void topic_2_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Topic 2 news: '%s'", msg.data.c_str());
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriberWithUniqueNetworkFlowEndpoints>());
  rclcpp::shutdown();
  return 0;
}
