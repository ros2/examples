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

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/Uint32.h>


template<typename T>
rclcpp::subscription::SubscriptionBase::SharedPtr subscribe(
  rclcpp::Node::SharedPtr node,
  typename rclcpp::subscription::Subscription<T>::CallbackType callback)
{
  auto sub = node->create_subscription<T>("topic_name", 10, callback);
  return sub;
}

void print_counter_data(const simple_msgs::Uint32::ConstPtr & msg)
{
  std::cout << "Got message #" << msg->data << std::endl;
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_subscriber");
  auto sub = subscribe<simple_msgs::Uint32>(node, print_counter_data);
  rclcpp::spin(node);
  return 0;
}
