// Copyright 2021, Apex.AI Inc.
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

#ifndef WAIT_SET__LISTENER_HPP_
#define WAIT_SET__LISTENER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "wait_set/visibility.h"

class Listener : public rclcpp::Node
{
public:
  WAIT_SET_PUBLIC explicit Listener(rclcpp::NodeOptions options);
  WAIT_SET_PUBLIC ~Listener() override;

private:
  void spin_wait_set();

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
  rclcpp::WaitSet wait_set_;
  std::thread thread_;
};

#endif  // WAIT_SET__LISTENER_HPP_
