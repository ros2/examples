// Copyright 2014 Open Source Robotics Foundation, Inc.
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
