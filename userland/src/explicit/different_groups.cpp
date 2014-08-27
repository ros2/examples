#include <iostream>

#include <rclcpp/rclcpp.hpp>

using rclcpp::callback_group::CallbackGroupType;

void on_timer_group1()
{
  std::cout << "In on_timer_group1." << std::endl;
}

void on_timer_group2()
{
  std::cout << "In on_timer_group2." << std::endl;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("my_node");

  auto g1 = node->create_callback_group(CallbackGroupType::MutuallyExclusive);
  auto g2 = node->create_callback_group(CallbackGroupType::Reentrant);

  auto timer1 = node->create_wall_timer(0.5_s, on_timer_group1, g1);
  auto timer2 = node->create_wall_timer(0.5_s, on_timer_group2, g1);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  return 0;
}
