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

#include <cinttypes>
#include <memory>
#include <chrono>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

std::shared_ptr<rclcpp::Node> g_node = nullptr;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("minimal_service");
  auto server = g_node->create_service<AddTwoInts>("add_two_ints",
    [&](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<AddTwoInts::Request> request,
      const std::shared_ptr<AddTwoInts::Response> response,
      const std::shared_ptr<rclcpp::Service<AddTwoInts>> service) // async when we received service pointer in the callback
    {

      std::thread t([=](){
        using namespace std::chrono_literals;
        RCLCPP_INFO(g_node->get_logger(), "thinking...");
        std::this_thread::sleep_for(1s);
        
        RCLCPP_INFO(g_node->get_logger(), "still thinking...");
        std::this_thread::sleep_for(1s);

        RCLCPP_INFO(g_node->get_logger(), "thinking some more...");
        std::this_thread::sleep_for(1s);

        RCLCPP_INFO(
          g_node->get_logger(),
          "request: %" PRId64 " + %" PRId64, request->a, request->b);
        response->sum = request->a + request->b;
        service->send_response(*request_header, *response);
      });
      t.detach();
    }
  );
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
