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

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using example_interfaces::srv::AddTwoInts;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the server. */

class MinimalService : public rclcpp::Node
{
public:
  MinimalService()
  : Node("minimal_service")
  {
    server_ = this->create_service<AddTwoInts>(
      "add_two_ints", std::bind(&MinimalService::service_callback, this, _1, _2));
  }

private:
  void service_callback(
    const AddTwoInts::Request::SharedPtr request,
    AddTwoInts::Response::SharedPtr response) const
  {
    RCLCPP_INFO(
      this->get_logger(),
      "request: %" PRId64 " + %" PRId64, request->a, request->b);
    response->sum = request->a + request->b;
  }
  rclcpp::Service<AddTwoInts>::SharedPtr server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalService>());
  rclcpp::shutdown();
  return 0;
}
