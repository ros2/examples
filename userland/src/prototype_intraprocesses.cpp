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

#include <iostream>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/msg/intraprocess.hpp>

struct Intraprocess_t
{
  std::string * data;
  int count; // # of receivers
};

void decrement(Intraprocess_t * meta)
{
  (meta->count)--;
  if (meta->count == 0) {
    std::cout << "------ remove the object!" << std::endl;
    delete meta->data;
  }
}

void callback(const simple_msgs::msg::Intraprocess::ConstSharedPtr & msg)
{
  Intraprocess_t * meta = (Intraprocess_t *) msg->ptr;
  std::cout << "------ Got message: " << *(meta->data) << std::endl;
  decrement(meta);
}

void monitor(std::string * s)
{
  while (1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "------ monitor string:" << *s << std::endl;
  }
}

void launch_subscriber(void)
{
  std::cout << "------ Creating subscriber:" << std::endl;
  auto node = rclcpp::Node::make_shared("prototype_intraprocess_sub");
  auto sub = node->create_subscription<simple_msgs::msg::Intraprocess>("intraprocess", 1000, callback);

  std::cout << "------ Launching subscriber:" << std::endl;
  rclcpp::spin(node);
}

void launch_publisher(Intraprocess_t * s)
{
  std::cout << "------ Creating publisher:" << std::endl;
  auto n = rclcpp::Node::make_shared("prototype_intraprocess_pub");
  auto p = n->create_publisher<simple_msgs::msg::Intraprocess>("intraprocess", 1000);
  simple_msgs::msg::Intraprocess::SharedPtr ros_msg;

  // Wait one second for the publisher to set up.
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  int number_of_msgs = 1;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < number_of_msgs; ++i) {
    ros_msg->ptr = (uint64_t)s;
    p->publish(ros_msg);
    // if (i % 100000 == 0) {
    std::cout << "------ published ros pointer to Intraprocess_t " << i << std::endl;
    // }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  auto end = std::chrono::steady_clock::now();

  std::cout << (end - start).count() << std::endl;
}

int main(int argc, char ** argv)
{
  std::string * s = new std::string("Message sent through ROS");
  Intraprocess_t meta;
  meta.data = s;
  meta.count = 1;

  // monitor
  std::thread monitor_thread(monitor, s);
  // publisher
  std::thread publisher_thread(launch_publisher, &meta);
  // subscriber
  launch_subscriber();
  publisher_thread.join();

  return 0;
}
