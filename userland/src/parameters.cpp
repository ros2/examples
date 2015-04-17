#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

template <typename ParameterTypeT>
ParameterTypeT
wait_for_future(rclcpp::node::Node::SharedPtr & node, std::shared_future<ParameterTypeT> & future)
{
  std::future_status status;
  do {
    rclcpp::spin_some(node);
    status = future.wait_for(std::chrono::milliseconds(100));
  } while (status != std::future_status::ready && rclcpp::ok());

  if (std::future_status::ready == status) {
    return future.get();
  }

  throw std::runtime_error("Invalid future status");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const std::string node_name("parameters_example");

  rclcpp::node::Node::SharedPtr node = rclcpp::Node::make_shared(node_name);

  auto f1 = node->async_has_parameter(node_name, rclcpp::parameter::ParameterQuery("foo"));
  bool found = wait_for_future<bool>(node, f1);
  std::cout << "Parameter (foo) found: " << found << std::endl;

  int64_t value1 = 1234;
  auto f2 = node->async_set_parameter(node_name, "foo", value1);
  wait_for_future<bool>(node, f2);
  std::cout << "Setting value: " << value1 << " for parameter (foo)" << std::endl;

  auto f3 = node->async_has_parameter(node_name, rclcpp::parameter::ParameterQuery("foo"));
  found = wait_for_future<bool>(node, f3);
  std::cout << "Parameter (foo) found: " << found << std::endl;

  const std::vector<rclcpp::parameter::ParameterContainer> kv = {
    rclcpp::parameter::ParameterContainer("foo", value1)};

  auto f4 = node->async_set_parameters(node_name, kv);
  wait_for_future<bool>(node, f4);

  auto f5 = node->async_get_parameter<int64_t>(node_name, "foo");
  int64_t value2 = wait_for_future<int64_t>(node, f5);
  std::cout << "Value for parameter (foo): " << value2 << std::endl;

  const std::vector<std::string> pn = { "foo" };
  auto f6 = node->async_get_parameters(node_name, pn);
  auto multi_value = wait_for_future< std::vector<rclcpp::parameter::ParameterContainer> >(node, f6);

  for(auto v : multi_value) {
    std::cout << "Value for parameter (foo) (multi value): " << v.get_value<int64_t>() << std::endl;
  }

  auto f7 = node->async_get_parameter<int64_t>(
    node_name, "foo", [] (std::shared_future<int64_t> f) {
      std::cout << "Value for parameter (foo) (callback): " << f.get() << std::endl; }
  );
  wait_for_future<int64_t>(node, f7);
  rclcpp::spin(node);

  return 0;
}
