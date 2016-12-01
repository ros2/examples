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
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

void on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // TODO(wjwwood): The message should have an operator<<, which would replace all of this.
  std::cout << "Parameter event:" << std::endl << " new parameters:" << std::endl;
  for (auto & new_parameter : event->new_parameters) {
    std::cout << "  " << new_parameter.name << std::endl;
  }
  std::cout << " changed parameters:" << std::endl;
  for (auto & changed_parameter : event->changed_parameters) {
    std::cout << "  " << changed_parameter.name << std::endl;
  }
  std::cout << " deleted parameters:" << std::endl;
  for (auto & deleted_parameter : event->deleted_parameters) {
    std::cout << "  " << deleted_parameter.name << std::endl;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("parameter_events");

  // TODO(esteve): Make the parameter service automatically start with the node.
  auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);

  auto parameters_client = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(node);

  // Setup callback for changes to parameters.
  auto sub = parameters_client->on_parameter_event(on_parameter_event);

  // Set several differnet types of parameters.
  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("foo", 2),
    rclcpp::parameter::ParameterVariant("bar", "hello"),
    rclcpp::parameter::ParameterVariant("baz", 1.45),
    rclcpp::parameter::ParameterVariant("foobar", true),
  });
  rclcpp::spin_until_future_complete(node, set_parameters_results);

  // Change the value of some of them.
  set_parameters_results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("foo", 3),
    rclcpp::parameter::ParameterVariant("bar", "world"),
  });
  rclcpp::spin_until_future_complete(node, set_parameters_results);

  // TODO(wjwwood): Create and use delete_parameter

  rclcpp::sleep_for(100ms);

  rclcpp::spin_some(node);

  return 0;
}
