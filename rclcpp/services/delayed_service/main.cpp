// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

class DelayedSumService : public rclcpp::Node
{
public:
  DelayedSumService()
  : Node("delayed_service")
  {
    // Declares a parameter for delaying (default to 2.0 seconds)
    this->declare_parameter("response_delay", 2.0);

    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints", std::bind(
                        &DelayedSumService::add_two_ints_callback, this, std::placeholders::_1,
                        std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "DelayedSumService is ready.");
  }

private:
  void add_two_ints_callback(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    // Gets parameter value
    double delay;
    this->get_parameter("response_delay", delay);

    auto result = request->a + request->b;
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Request:" << request->a << " + " << request->b << " delayed " << delay << " seconds");

    // Simulates the delay
    std::this_thread::sleep_for(std::chrono::duration<double>(delay));

    response->sum = result;
    RCLCPP_INFO_STREAM(this->get_logger(), "Response: " << result);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DelayedSumService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
