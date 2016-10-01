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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("set_and_get_parameters_async");

  // TODO(wjwwood): Make the parameter service automatically start with the node.
  auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);

  auto parameters_client = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(node);

  // Set several different types of parameters.
  auto results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("foo", 2),
    rclcpp::parameter::ParameterVariant("bar", "hello"),
    rclcpp::parameter::ParameterVariant("baz", 1.45),
    rclcpp::parameter::ParameterVariant("foobar", true),
  });
  // Wait for the results.
  if (rclcpp::spin_until_future_complete(node, results) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    printf("set_parameters service call failed. Exiting example.\n");
    return -1;
  }
  // Check to see if they were set.
  for (auto & result : results.get()) {
    if (!result.successful) {
      std::cerr << "Failed to set parameter: " << result.reason << std::endl;
    }
  }

  // Get a few of the parameters just set.
  auto parameters = parameters_client->get_parameters({"foo", "baz"});
  if (rclcpp::spin_until_future_complete(node, parameters) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    printf("get_parameters service call failed. Exiting example.\n");
    return -1;
  }
  for (auto & parameter : parameters.get()) {
    std::cout << "Parameter name: " << parameter.get_name() << std::endl;
    std::cout << "Parameter value (" << parameter.get_type_name() << "): " <<
      parameter.value_to_string() << std::endl;
  }

  return 0;
}
