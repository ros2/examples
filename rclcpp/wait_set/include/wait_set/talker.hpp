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

#ifndef WAIT_SET__TALKER_HPP_
#define WAIT_SET__TALKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
#include "wait_set/visibility.h"

class Talker : public rclcpp::Node
{
public:
  WAIT_SET_PUBLIC explicit Talker(rclcpp::NodeOptions options);

private:
  size_t count_;
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // WAIT_SET__TALKER_HPP_
