// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <memory>
// #include <string>
// #include <thread>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // auto node = std::make_shared<rclcpp::Node>("wait_set_example_node");

  // Normal WaitSet example
  {
    auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
    auto guard_condition2 = std::make_shared<rclcpp::GuardCondition>();

    rclcpp::WaitSet wait_set(std::vector<rclcpp::GuardCondition::SharedPtr>{guard_condition});
    wait_set.add_guard_condition(guard_condition2);

    {
      auto wait_result = wait_set.wait(std::chrono::seconds(1));
      assert(wait_result.kind() == rclcpp::WaitResultKind::Timeout);
    }

    wait_set.remove_guard_condition(guard_condition2);

    {
      // still fails with timeout
      auto wait_result = wait_set.wait(std::chrono::seconds(1));
      assert(wait_result.kind() == rclcpp::WaitResultKind::Timeout);
    }

    wait_set.remove_guard_condition(guard_condition);

    {
      // now fails (fast) with empty
      auto wait_result = wait_set.wait(std::chrono::seconds(1));
      assert(wait_result.kind() == rclcpp::WaitResultKind::Empty);
    }
  }

  // StaticWaitSet example
  {
    auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
    auto guard_condition2 = std::make_shared<rclcpp::GuardCondition>();

    rclcpp::StaticWaitSet<1> static_wait_set(
      std::array<rclcpp::GuardCondition::SharedPtr, 1>{{guard_condition}});
    // Note: The following line will result in a compiler error, since the
    //   static storage policy prevents editing after construction.
    // static_wait_set.add_guard_condition(guard_condition2);
    // static_wait_set.remove_guard_condition(guard_condition2);
    (void)guard_condition2;

    {
      auto wait_result = static_wait_set.wait(std::chrono::seconds(1));
      assert(wait_result.kind() == rclcpp::WaitResultKind::Timeout);
    }
  }

  // ThreadSafeWaitSet example
  // {
  //   auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
  //   auto guard_condition2 = std::make_shared<rclcpp::GuardCondition>();

  //   rclcpp::WaitSet wait_set({{guard_condition}});
  //   wait_set.add_guard_condition(guard_condition2);
  // }

  return 0;
}
