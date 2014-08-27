#include <iostream>

#include <rclcpp/rclcpp.hpp>

void on_timer_group1()
{
  std::cout << "In on_timer_group1." << std::endl;
}

void on_timer_group2()
{
  std::cout << "In on_timer_group2." << std::endl;
}

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);

  auto group1 = rclcpp::create_callback_group(
    "group1", rclcpp::callback_group::NonThreadSafe);
  auto group2 = rclcpp::create_callback_group(
    "group2", rclcpp::callback_group::ThreadSafe);

  auto timer = group1->create_timer(0.5, on_timer_group1);
  auto timer = group2->create_timer(0.5, on_timer_group2);

  auto executor = rclcpp::executors::MultiThreadedExecutor();
  executor.register_group(group1);
  executor.register_group(group2);

  executor.exec();

  return 0;
}
