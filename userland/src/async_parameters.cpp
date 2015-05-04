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

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("parameters");

  auto parameters_client = std::make_shared<rclcpp::parameter::AsyncParametersClient>(node);

  auto parameters = {
    rclcpp::parameter::ParameterVariant("foo", 2),
    rclcpp::parameter::ParameterVariant("bar", "hello"),
    rclcpp::parameter::ParameterVariant("baz", 1.45),
    rclcpp::parameter::ParameterVariant("foobar", true),
  };

  auto f1 = parameters_client->set_parameters(
    parameters, [](std::shared_future<std::vector<rcl_interfaces::SetParametersResult>> f) {
    for (auto v : f.get()) {
      if (!v.successful) {
        std::cerr << v.reason << std::endl;
      }
    }
  });

  rclcpp::spin_until_future_complete(node, f1);

  auto f2 = parameters_client->get_parameters({{"foo", "baz"}});

  auto values_f2 = rclcpp::spin_until_future_complete(node, f2).get();

  for (auto p : values_f2) {
    std::cout << "Parameter name: " << p.get_name() << std::endl;
    std::cout << "Parameter value: ";
    switch (p.get_type()) {
      case rclcpp::parameter::ParameterType::PARAMETER_BOOL:
        std::cout << "(bool): " << p.get_value<bool>() ? "true" : "false";
        break;
      case rclcpp::parameter::ParameterType::PARAMETER_INTEGER:
        std::cout << "(integer): " << p.get_value<int64_t>();
        break;
      case rclcpp::parameter::ParameterType::PARAMETER_DOUBLE:
        std::cout << "(double): " << p.get_value<double>();
        break;
      case rclcpp::parameter::ParameterType::PARAMETER_STRING:
        std::cout << "(string): " << p.get_value<std::string>();
        break;
      case rclcpp::parameter::ParameterType::PARAMETER_BYTES:
        std::cout << "(bytes)";
        break;
      case rclcpp::parameter::ParameterType::PARAMETER_NOT_SET:
      default:
        std::cout << "INVALID";
    }
    std::cout << std::endl;
  }

  auto f3 = parameters_client->list_parameters({{"foo", "bar"}}, 10);

  auto result_f3 = rclcpp::spin_until_future_complete(node, f3).get();
  for (auto parameter_name : result_f3.parameter_names) {
    std::cout << "Parameter name: " << parameter_name << std::endl;
  }
  for (auto parameter_prefix : result_f3.parameter_prefixes) {
    std::cout << "Parameter prefix: " << parameter_prefix << std::endl;
  }


  rclcpp::spin(node);

  return 0;
}
