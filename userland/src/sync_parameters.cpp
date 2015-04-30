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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/AllBuiltinTypes.h>
#include <simple_msgs/AllDynamicArrayTypes.h>
#include <simple_msgs/AllPrimitiveTypes.h>
#include <simple_msgs/AllStaticArrayTypes.h>
#include <simple_msgs/Nested.h>
#include <simple_msgs/String.h>
#include <simple_msgs/Uint32.h>

#include <userland_msgs/AddTwoInts.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("parameters");

  auto sync_parameters_client = std::make_shared<rclcpp::parameter::SyncParametersClient>(node);

  auto parameters = {
    rclcpp::parameter::ParameterVariant("foo", int64_t(2)),
    rclcpp::parameter::ParameterVariant("bar", std::string("hello")),
    rclcpp::parameter::ParameterVariant("baz", double(1.45)),
    rclcpp::parameter::ParameterVariant("foobar", true),
  };

  auto results1 = sync_parameters_client->set_parameters(parameters);
  for (auto v : results1) {
    if (!v.successful) {
      std::cerr << v.reason << std::endl;
    }
  }

  auto results2 = sync_parameters_client->get_parameters({{"foo", "baz"}});

  for (auto p : results2) {
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

  auto results3 = sync_parameters_client->list_parameters({{"foo", "bar"}}, 10);

  for (auto parameter_name : results3.parameter_names) {
    std::cout << "Parameter name: " << parameter_name << std::endl;
  }
  for (auto parameter_prefix : results3.parameter_prefixes) {
    std::cout << "Parameter prefix: " << parameter_prefix << std::endl;
  }

  rclcpp::spin(node);

  return 0;
}
