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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("parameters");

  auto sync_parameters_client = std::make_shared<rclcpp::parameter_client::SyncParametersClient>(node);

  // Set several differnet types of parameters.
  auto set_parameters_results = sync_parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("foo", 2),
    rclcpp::parameter::ParameterVariant("bar", "hello"),
    rclcpp::parameter::ParameterVariant("baz", 1.45),
    rclcpp::parameter::ParameterVariant("foobar", true),
  });
  // Check to see if they were set.
  for (auto & result : set_parameters_results) {
    if (!result.successful) {
      std::cerr << result.reason << std::endl;
    }
  }

  // Get a few of the parameters just set.
  for (auto & parameter : sync_parameters_client->get_parameters({"foo", "baz"})) {
    std::cout << "Parameter name: " << parameter.get_name() << std::endl;
    std::cout << "Parameter value (" << parameter.get_type_name() << "): "
              << parameter.to_string() << std::endl;
  }

  // List the details of a few parameters up to a namespace depth of 10.
  auto parameters_and_prefixes = sync_parameters_client->list_parameters({"foo", "bar"}, 10);
  for (auto & name : parameters_and_prefixes.names) {
    std::cout << "Parameter name: " << name << std::endl;
  }
  for (auto & prefix : parameters_and_prefixes.prefixes) {
    std::cout << "Parameter prefix: " << prefix << std::endl;
  }

  return 0;
}
