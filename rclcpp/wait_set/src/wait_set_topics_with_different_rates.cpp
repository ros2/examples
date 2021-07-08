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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* For this example, we will be creating three talkers publishing the topics A, B, C at different
 * rates. The messages are handled by a wait-set loop which handles topics A and B together on a
 * using topic B as trigger condition. Topic C is handled independently using the topic C
 * itself as a trigger condition. */

class Talker : public rclcpp::Node
{
public:
  Talker(
    const std::string & node_name,
    const std::string & topic_name,
    const std::string & message_data,
    std::chrono::nanoseconds period)
  : Node(node_name)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
    auto timer_callback =
      [this, message_data]() -> void {
        std_msgs::msg::String message;
        message.data = message_data;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(period, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int32_t main(const int32_t argc, char ** const argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("wait_set_listener");
  auto do_nothing = [](std_msgs::msg::String::UniquePtr) {assert(false);};

  auto sub1 = node->create_subscription<std_msgs::msg::String>("topicA", 10, do_nothing);
  auto sub2 = node->create_subscription<std_msgs::msg::String>("topicB", 10, do_nothing);
  auto sub3 = node->create_subscription<std_msgs::msg::String>("topicC", 10, do_nothing);

  rclcpp::WaitSet wait_set({{sub1}, {sub2}, {sub3}});

  // Create three talkers publishing in topics A, B, and C with different publishing rates
  auto talkerA = std::make_shared<Talker>("TalkerA", "topicA", "A", 1500ms);
  auto talkerB = std::make_shared<Talker>("TalkerB", "topicB", "B", 2000ms);
  auto talkerC = std::make_shared<Talker>("TalkerC", "topicC", "C", 3000ms);

  // Create an executor to spin the talkers in a separate thread
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(talkerA);
  exec.add_node(talkerB);
  exec.add_node(talkerC);
  auto publisher_thread = std::thread([&exec]() {exec.spin();});

  while (rclcpp::ok()) {
    const auto wait_result = wait_set.wait(3s);
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      bool sub2_has_data = wait_result.get_wait_set().get_rcl_wait_set().subscriptions[1U];
      bool sub3_has_data = wait_result.get_wait_set().get_rcl_wait_set().subscriptions[2U];

      // topic A and B handling
      // Note only topic B is used as a trigger condition
      if (sub2_has_data) {
        std_msgs::msg::String msg1;
        std_msgs::msg::String msg2;
        rclcpp::MessageInfo msg_info;
        std::string handled_data;

        if (sub2->take(msg2, msg_info)) {
          // since topic A is published at a faster rate we expect to take multiple messages
          while (sub1->take(msg1, msg_info)) {
            handled_data.append(msg1.data);
          }
          handled_data.append(msg2.data);
          RCLCPP_INFO(node->get_logger(), "I heard: '%s'", handled_data.c_str());
        } else {
          RCLCPP_ERROR(node->get_logger(), "An invalid message from topic B was received.");
        }
      }

      // topic C handling
      if (sub3_has_data) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        if (sub3->take(msg, msg_info)) {
          RCLCPP_INFO(node->get_logger(), "I heard: '%s'", msg.data.c_str());
        } else {
          RCLCPP_ERROR(node->get_logger(), "An invalid message from topic C was received.");
        }
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Wait-set failed with timeout");
      }
    }
  }

  rclcpp::shutdown();
  publisher_thread.join();
  return 0;
}
