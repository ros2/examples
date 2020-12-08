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

class MinimalSubscriberWithUniqueNetworkFlow : public rclcpp::Node
{
public:
  MinimalSubscriberWithUniqueNetworkFlow()
  : Node("minimal_subscriber_with_unique_network_flow")
  {
    // Enable unique network flow via options
    auto options_1 = rclcpp::SubscriptionOptions();
    options_1.unique_network_flow = true;

    subscription_1_ = this->create_subscription<std_msgs::msg::String>(
      "topic_1", 10, std::bind(
        &MinimalSubscriberWithUniqueNetworkFlow::topic_1_callback, this,
        _1), options_1);

    // Unique network flow is disabled by default
    auto options_2 = rclcpp::SubscriptionOptions();
    subscription_2_ = this->create_subscription<std_msgs::msg::String>(
      "topic_2", 10, std::bind(
        &MinimalSubscriberWithUniqueNetworkFlow::topic_2_callback, this,
        _1), options_2);

    // Print network flows and check for uniqueness
    auto network_flows_1 = subscription_1_->get_network_flow();
    auto network_flows_2 = subscription_2_->get_network_flow();
    print_network_flows(network_flows_1);
    print_network_flows(network_flows_2);
    if (!are_network_flows_unique(network_flows_1, network_flows_2)) {
      RCLCPP_ERROR(this->get_logger(), "Network flows across subscriptions are not unique");
    }
  }

private:
  void topic_1_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Topic 1 news: '%s'", msg->data.c_str());
  }
  void topic_2_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Topic 2 news: '%s'", msg->data.c_str());
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
      this->get_logger(), "Subscription created with network flows: '%s'",
      stream.str().c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriberWithUniqueNetworkFlow>());
  rclcpp::shutdown();
  return 0;
}
