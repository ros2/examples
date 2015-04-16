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

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <userland_msgs/AddTwoInts.h>

template<typename T>
int request(
  rclcpp::Node::SharedPtr node,
  std::function<void(typename T::Request::Ptr &, size_t)> set_data_func,
  size_t number_of_requests)
{
  int rc = 1;
  auto client = node->create_client<T>("service_name");
  typename T::Request::Ptr ros_request(new typename T::Request());

  auto start = std::chrono::steady_clock::now();
  size_t i = 1;
  while (rclcpp::ok() && i <= number_of_requests) {
    set_data_func(ros_request, i);
    auto f = client->async_send_request(ros_request);
    std::cout << "requested service #" << i << std::endl;

    auto wait_for_response_until = std::chrono::steady_clock::now() +
      std::chrono::seconds(1);
    std::future_status status;
    do {
      rclcpp::spin_some(node);
      status = f.wait_for(std::chrono::milliseconds(100));
    } while (
      status != std::future_status::ready && rclcpp::ok() &&
      std::chrono::steady_clock::now() < wait_for_response_until
    );

    if (std::future_status::ready == status) {
      std::cout << "Got response #" << i << std::endl;
      rc = 0;
      if (f.get()->sum / 2 != i) {
        std::cout << "Got wrong response " << f.get()->sum << " for request " << i << std::endl;
        rc = 2;
      }
      break;
    }
    ++i;
  }
  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<float> diff = (end - start);
  std::cout << "Runtime: " << diff.count() << " seconds" << std::endl;

  return rc;
}

void set_data(userland_msgs::AddTwoInts::Request::Ptr & ros_request, size_t i)
{
  ros_request->a = i;
  ros_request->b = i;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_client");

  return request<userland_msgs::AddTwoInts>(node, &set_data, 10);
}
