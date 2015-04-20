#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const std::string node_name("parameters_example");

  auto node = rclcpp::Node::make_shared(node_name);

  auto f1 = node->async_has_parameter(node_name, "foo");
  bool found = rclcpp::spin_until_future_complete<bool>(node, f1).get();
  std::cout << "Parameter (foo) found: " << (found ? "true" : "false") << std::endl;

  int64_t value1 = 1234;
  auto f2 = node->async_set_parameter(node_name, "foo", value1);
  rclcpp::spin_until_future_complete<bool>(node, f2);
  std::cout << "Setting value: " << value1 << " for parameter (foo)" << std::endl;

  auto f3 = node->async_has_parameter(node_name, "foo");
  found = rclcpp::spin_until_future_complete<bool>(node, f3).get();
  std::cout << "Parameter (foo) found: " << (found ? "true" : "false") << std::endl;

  const std::vector<rclcpp::parameter::ParameterContainer> kv = {
    rclcpp::parameter::ParameterContainer("foo", value1)
  };

  auto f4 = node->async_set_parameters(node_name, kv);
  rclcpp::spin_until_future_complete<bool>(node, f4);

  auto f5 = node->async_get_parameter<int64_t>(node_name, "foo");
  int64_t value2 = rclcpp::spin_until_future_complete<int64_t>(node, f5).get();
  std::cout << "Value for parameter (foo): " << value2 << std::endl;

  const std::vector<std::string> pn = {"foo"};
  auto f6 = node->async_get_parameters(node_name, pn);
  auto multi_value = rclcpp::spin_until_future_complete<std::vector<rclcpp::parameter::ParameterContainer>>(node, f6).get();

  for (auto v : multi_value) {
    std::cout << "Value for parameter (foo) (multi value): " << v.get_value<int64_t>() << std::endl;
  }

  auto f7 = node->async_get_parameter<int64_t>(
    node_name, "foo", [](std::shared_future<int64_t> f) {
    std::cout << "Value for parameter (foo) (callback): " << f.get() << std::endl;
  }
    );
  rclcpp::spin_until_future_complete<int64_t>(node, f7);
  rclcpp::spin(node);

  return 0;
}
