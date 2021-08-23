// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using example_interfaces::srv::AddTwoInts;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the client. */

class MinimalClient : public rclcpp::Node
{
public:
  MinimalClient()
  : Node("minimal_client")
  {
    client_ = this->create_client<AddTwoInts>("add_two_ints");
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = 41;
    request->b = 1;
    auto result_future = client_->async_send_request(
      request, std::bind(&MinimalClient::service_callback, this, _1));
  }

private:
  void service_callback(rclcpp::Client<AddTwoInts>::SharedFutureWithRequest result_future) const
  {
    auto result = result_future.get();
    auto request = result.first;
    auto response = result.second;
    RCLCPP_INFO(
      this->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,
      request->a, request->b, response->sum);
    rclcpp::shutdown();
  }
  rclcpp::Client<AddTwoInts>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalClient>());
  rclcpp::shutdown();
  return 0;
}
