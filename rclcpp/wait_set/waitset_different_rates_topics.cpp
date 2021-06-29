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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <random>

/* For this example, we will be creating a publishing node with three publishers which will
 * publish the topics A, B, C in random order each time. The order in which the messages are
 * handled is defined deterministically directly by the user in the code. That is, in this example
 * we always take and process the data in the same order  A, B, C regardless of the arrival order.
 */
class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker"),
    pub1_(this->create_publisher<std_msgs::msg::String>("topicA", 10)),
    pub2_(this->create_publisher<std_msgs::msg::String>("topicB", 10)),
    pub3_(this->create_publisher<std_msgs::msg::String>("topicC", 10)),
    count_(0U)
  {
  }

  void run()
  {
    std_msgs::msg::String msg1, msg2, msg3;
    msg1.data = "A";
    msg2.data = "B";
    msg3.data = "C";

    while (rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Publishing msg1: %s", msg1.data.c_str());
      pub1_->publish(msg1);
      if ( (count_ % 4) == 0) {
        RCLCPP_INFO(this->get_logger(), "Publishing msg3: %s", msg3.data.c_str());
        pub3_->publish(msg3);
      }

      if ( (count_ % 2) == 0) {
        RCLCPP_INFO(this->get_logger(), "Publishing msg2: %s", msg2.data.c_str());
        pub2_->publish(msg2);
      }
      ++count_;
      std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub3_;
  size_t count_;
};

int32_t main(const int32_t argc, char ** const argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("waitset_node");
  auto do_nothing = [](std_msgs::msg::String::UniquePtr) {assert(false);};
  size_t count{0U};

  auto sub1 = node->create_subscription<std_msgs::msg::String>("topicA", 10, do_nothing);
  auto sub2 = node->create_subscription<std_msgs::msg::String>("topicB", 10, do_nothing);
  auto sub3 = node->create_subscription<std_msgs::msg::String>("topicC", 10, do_nothing);

  rclcpp::WaitSet wait_set({{sub1}, {sub2}, {sub3}});

  // Creates a random publisher and starts publishing in another thread
  Talker publisher_node;
  auto publisher_thread = std::thread([&publisher_node]() {publisher_node.run();});

  while (rclcpp::ok()) {
    // Waiting up to 5s for a message to arrive
    const auto wait_result = wait_set.wait(std::chrono::seconds(5));

    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      bool sub2_has_data = wait_result.get_wait_set().get_rcl_wait_set().subscriptions[1U];
      if (sub2_has_data) {
        std_msgs::msg::String msg1;
        std_msgs::msg::String msg2;
        std_msgs::msg::String msg3;
        rclcpp::MessageInfo msg_info;

        // the receiving order is not relevant, the messages are taken in a user-defined order
        bool msg1_is_valid = sub1->take(msg1, msg_info);
        bool msg2_is_valid = sub2->take(msg2, msg_info);
        bool msg3_is_valid = sub3->take(msg3, msg_info);

        // we process all the messages only if all are valid
        if (msg1_is_valid && msg2_is_valid) {
          RCLCPP_INFO(
            node->get_logger(), "Handle: %s%s", msg1.data.c_str(),
            msg2.data.c_str());
        } else {
          RCLCPP_INFO(node->get_logger(), "An invalid message was received.");
        }
        if (msg3_is_valid) {
          RCLCPP_INFO(node->get_logger(), "Handle: %s", msg3.data.c_str());
        }

        ++count;
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      RCLCPP_INFO(node->get_logger(), "No message received after 5s.");
    } else {
      RCLCPP_INFO(node->get_logger(), "Wait-set failed.");
    }
  }

  rclcpp::shutdown();
  publisher_thread.join();
  return 0;
}
