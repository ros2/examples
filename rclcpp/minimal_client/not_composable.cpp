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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "examples_rclcpp_minimal_service/srv/add_two_ints.hpp"

using Service = examples_rclcpp_minimal_service::srv::AddTwoInts;
using Request = Service::Request;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("minimal_subscriber");
  auto client = node->create_client<Service>("service");
  while (!client->wait_for_service(1_s)) {
    if (!rclcpp::ok()) {
      printf("client interrupted while waiting for service to appear.\n");
      return 1;
    }
    printf("waiting for service to appear...\n");
  }
  auto request = std::make_shared<Request>();
  request->a = 41;
  request->b = 1;
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    printf("service call failed :(\n");
    return 1;
  }
  auto result = result_future.get();
  printf("result of sending service request for %ld + %ld = %ld\n",
    request->a, request->b, result->sum);
  return 0;
}
