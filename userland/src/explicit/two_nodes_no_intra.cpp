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

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/String.h>

void on_message(const simple_msgs::String::SharedPtr &msg)
{
  std::cout << *msg << std::endl;
}

void on_timer(const rclcpp::TimerEvent &event,
              rclcpp::Publisher::UniquePtr &publisher,
              int &i)
{
  simple_msgs::String::UniquePtr msg;
  msg->data = "Hello World: " + std::to_string(i++);
  if (publisher) {
    publisher->publish(msg);
  } else {
    RCLCPP_ERROR("Invalid publisher!");
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // By putting nodes in different contexts, prevents them from using intra-pc
  auto context1 = rclcpp::context::get_shared_global_default_context();
  auto context2 = std::make_shared(new rclcpp::context::DefaultContext());

  auto node1 = rclcpp::Node::create_shared_node("node1", context1);
  auto node2 = rclcpp::Node::create_shared_node("node2", context2);

  auto publisher = node1->create_publisher<simple_msgs::String>("/chatter", 7);
  auto subscription = \
    node2->create_subscription<simple_msgs::String>("/chatter", 7, on_message);

  int i = 0;
  auto timer_bound = std::bind(on_timer, _1, std::ref(publisher), std::ref(i));
  auto timer = node1->create_timer(0.5, timer_bound);

  // auto executor = rclcpp::executors::SingleThreadedExecutor();
  auto executor = rclcpp::executors::MultiThreadedExecutor();

  executor.register_node(node1);
  executor.register_node(node2);

  executor.exec();

  return 0;
}
