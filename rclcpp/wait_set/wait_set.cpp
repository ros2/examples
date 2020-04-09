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

#include <cassert>
#include <memory>
// #include <string>
// #include <thread>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto do_nothing = [](std_msgs::msg::String::UniquePtr) {assert(false);};
  auto add_two_ints =
    [](
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>,
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>)
    {assert(false);};

  auto node = std::make_shared<rclcpp::Node>("wait_set_example_node");

  auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
  auto guard_condition2 = std::make_shared<rclcpp::GuardCondition>();

  auto sub = node->create_subscription<std_msgs::msg::String>("~/chatter", 10, do_nothing);
  rclcpp::SubscriptionOptions so;
  so.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  auto sub2 = node->create_subscription<std_msgs::msg::String>("~/chatter", 10, do_nothing, so);

  auto pub = node->create_publisher<std_msgs::msg::String>("~/chatter", 10);
  rclcpp::PublisherOptions po;
  po.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  auto pub2 = node->create_publisher<std_msgs::msg::String>("~/chatter", 10, po);

  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("~/add_two_ints");
  auto service =
    node->create_service<example_interfaces::srv::AddTwoInts>("~/add_two_ints", add_two_ints);

  rclcpp::WaitSet wait_set(
    std::vector<rclcpp::WaitSet::SubscriptionEntry>{{sub}},
    std::vector<rclcpp::GuardCondition::SharedPtr>{guard_condition});
  wait_set.add_subscription(sub2);
  wait_set.add_guard_condition(guard_condition2);

  {
    auto wait_result = wait_set.wait(std::chrono::seconds(1));
    assert(wait_result.kind() == rclcpp::WaitResultKind::Timeout);
  }

  guard_condition->trigger();

  {
    auto wait_result = wait_set.wait(std::chrono::seconds(1));
    assert(wait_result.kind() == rclcpp::WaitResultKind::Ready);
    assert(wait_result.get_wait_set().get_rcl_wait_set().guard_conditions[0] != nullptr);
    assert(wait_result.get_wait_set().get_rcl_wait_set().guard_conditions[1] == nullptr);
    assert(wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0] == nullptr);
    assert(wait_result.get_wait_set().get_rcl_wait_set().subscriptions[1] == nullptr);
  }

  wait_set.remove_guard_condition(guard_condition2);

  {
    // still fails with timeout
    auto wait_result = wait_set.wait(std::chrono::seconds(1));
    assert(wait_result.kind() == rclcpp::WaitResultKind::Timeout);
  }

  wait_set.remove_guard_condition(guard_condition);

  {
    // still fails with timeout
    auto wait_result = wait_set.wait(std::chrono::seconds(1));
    assert(wait_result.kind() == rclcpp::WaitResultKind::Timeout);
  }

  wait_set.remove_subscription(sub2);

  {
    // still fails with timeout
    auto wait_result = wait_set.wait(std::chrono::seconds(1));
    assert(wait_result.kind() == rclcpp::WaitResultKind::Timeout);
  }

  // auto pub = node->create_publisher<std_msgs::msg::String>("~/chatter", 1);
  pub->publish(std_msgs::msg::String().set__data("test"));

  {
    auto wait_result = wait_set.wait(std::chrono::seconds(1));
    assert(wait_result.kind() == rclcpp::WaitResultKind::Ready);
    assert(wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0] != nullptr);
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;
    assert(sub->take(msg, msg_info));
    assert(msg.data == "test");
  }

  wait_set.remove_subscription(sub);

  {
    // now fails (fast) with empty
    auto wait_result = wait_set.wait(std::chrono::seconds(1));
    assert(wait_result.kind() == rclcpp::WaitResultKind::Empty);
  }

  return 0;
}
