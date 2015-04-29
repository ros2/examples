// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <userland_msgs/AddTwoInts.h>

void add(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<userland_msgs::AddTwoInts::Request> request,
  std::shared_ptr<userland_msgs::AddTwoInts::Response> response)
{
  std::cout << "Got request #" << request_header->sequence_number << std::endl;
  response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_server");

  node->create_service<userland_msgs::AddTwoInts>("service_name",
    // TODO(wjwwood): find a more elegant way to avoid the "ambiguous overloaded function call"
    // See: https://github.com/ros2/rmw_connext/issues/26
    static_cast<rclcpp::service::Service<userland_msgs::AddTwoInts>::CallbackWithHeaderType>(add));

  rclcpp::spin(node);

  return 0;
}
