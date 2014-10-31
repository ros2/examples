#include <functional>
#include <iostream>
#include <thread>

#include <rclcpp/rclcpp.hpp>

using rclcpp::callback_group::CallbackGroupType;
using std::chrono::steady_clock;
typedef std::chrono::duration<float> floating_seconds;

void on_timer_group1()
{
  auto this_id = rclcpp::thread_id;
  auto start = steady_clock::now();
  std::cout << "[1:" << this_id << "] Start" << std::endl;
  rclcpp::sleep_for(0.001_s);
  auto diff = steady_clock::now() - start;
  std::cout << "[1:" << this_id << "] Stop after "
            << std::chrono::duration_cast<floating_seconds>(diff).count()
            << std::endl;
}

void on_timer_group2()
{
  auto this_id = rclcpp::thread_id;
  auto start = steady_clock::now();
  std::cout << "[2:" << this_id << "] Start" << std::endl;
  rclcpp::sleep_for(0.001_s);
  auto diff = steady_clock::now() - start;
  std::cout << "[2:" << this_id << "] Stop after "
            << std::chrono::duration_cast<floating_seconds>(diff).count()
            << std::endl;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("my_node");

  // auto g1 = node->create_callback_group(CallbackGroupType::MutuallyExclusive);
  auto g2 = node->create_callback_group(CallbackGroupType::MutuallyExclusive);
  // auto g2 = node->create_callback_group(CallbackGroupType::Reentrant);

  // auto timer1 = node->create_wall_timer(2.0_s, on_timer_group1, g1);
  auto timer2 = node->create_wall_timer(0.25_s, on_timer_group2, g2);
  // auto timer3 = node->create_wall_timer(0.001_s, on_timer_group1, g2);

  // rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  return 0;
}
