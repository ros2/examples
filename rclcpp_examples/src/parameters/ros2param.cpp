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

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#define USAGE "USAGE:\n  ros2param get <node/variable>\n  ros2param set <node/variable> <value>"

typedef enum
{
  PARAM_NONE,
  PARAM_GET,
  PARAM_SET,
} param_operation_t;

rclcpp::parameter::ParameterVariant
parse_args(int argc, char ** argv, std::string & remote_node, param_operation_t & op)
{
  if (argc < 3) {
    return rclcpp::parameter::ParameterVariant();
  }

  std::string verb = argv[1];
  std::string name = argv[2];
  size_t slash = name.find('/');
  if ((slash == std::string::npos) ||
    (slash == 0) ||
    (slash == (name.size() - 1)))
  {
    return rclcpp::parameter::ParameterVariant();
  }
  remote_node = name.substr(0, slash);
  std::string variable = name.substr(slash + 1, name.size() - slash - 1);

  if ((verb == "get") && (argc == 3)) {
    op = PARAM_GET;
    return rclcpp::parameter::ParameterVariant(variable, 0);
  } else if ((verb == "set") && (argc == 4)) {
    op = PARAM_SET;
    std::string value = argv[3];
    char * endptr;
    int l = strtol(value.c_str(), &endptr, 10);
    if ((errno == 0) && (*endptr == '\0')) {
      return rclcpp::parameter::ParameterVariant(variable, l);
    } else {
      errno = 0;
      double d = strtod(value.c_str(), &endptr);
      if ((errno == 0) && (*endptr == '\0')) {
        return rclcpp::parameter::ParameterVariant(variable, d);
      } else {
        if ((value == "true") || (value == "True")) {
          return rclcpp::parameter::ParameterVariant(variable, true);
        } else if ((value == "false") || (value == "False")) {
          return rclcpp::parameter::ParameterVariant(variable, false);
        } else {
          return rclcpp::parameter::ParameterVariant(variable, value);
        }
      }
    }
  } else {
    return rclcpp::parameter::ParameterVariant();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string remote_node;
  param_operation_t op = PARAM_NONE;

  auto var = parse_args(argc, argv, remote_node, op);

  if (op == PARAM_NONE) {
    std::cout << USAGE << std::endl;
    return 1;
  }

  auto node = rclcpp::Node::make_shared("ros2param");
  auto parameters_client =
    std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(node, remote_node);

  if (op == PARAM_GET) {
    auto get_parameters_result = parameters_client->get_parameters({var.get_name()});
    auto get_result = rclcpp::spin_until_future_complete(
      node, get_parameters_result, std::chrono::milliseconds(1000));
    if ((get_result != rclcpp::executor::FutureReturnCode::SUCCESS) ||
      (get_parameters_result.get().size() != 1) ||
      (get_parameters_result.get()[0].get_type() == rclcpp::parameter::PARAMETER_NOT_SET))
    {
      std::cout << "Failed to get parameter" << std::endl;
      return 1;
    } else {
      auto result = get_parameters_result.get()[0];
      if (result.get_type() == rclcpp::parameter::PARAMETER_BOOL) {
        std::cout << result.get_value<bool>() << std::endl;
      } else if (result.get_type() == rclcpp::parameter::PARAMETER_INTEGER) {
        std::cout << result.get_value<int>() << std::endl;
      } else if (result.get_type() == rclcpp::parameter::PARAMETER_DOUBLE) {
        std::cout << result.get_value<double>() << std::endl;
      } else if (result.get_type() == rclcpp::parameter::PARAMETER_STRING) {
        std::cout << result.get_value<std::string>() << std::endl;
      } else if (result.get_type() == rclcpp::parameter::PARAMETER_BYTES) {
        std::cout << "BYTES type not implemented" << std::endl;
        return 1;
      }
    }
  } else if (op == PARAM_SET) {
    auto set_parameters_result = parameters_client->set_parameters({var});
    auto set_result = rclcpp::spin_until_future_complete(
      node, set_parameters_result, std::chrono::milliseconds(1000));
    if (set_result != rclcpp::executor::FutureReturnCode::SUCCESS) {
      std::cout << "Failed to set parameter" << std::endl;
    }
  } else {
    std::cout << USAGE << std::endl;
    return 1;
  }
  return 0;
}
