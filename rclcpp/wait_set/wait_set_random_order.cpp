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
class RandomPublisher : public rclcpp::Node
{
public:
  RandomPublisher()
  : Node("random_publisher"),
    pub1_(this->create_publisher<std_msgs::msg::String>("topicA", 10)),
    pub2_(this->create_publisher<std_msgs::msg::String>("topicB", 10)),
    pub3_(this->create_publisher<std_msgs::msg::String>("topicC", 10)),
    count_(0U)
  {
  }

  void run()
  {
    std_msgs::msg::String msg1, msg2, msg3;
    std::vector<std::function<void()>> publisher_functions;
    std::vector<std::string> msgs_data{"A", "B", "C"};
    std::vector<int> publish_order{0, 1, 2};
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine e(seed);

    msg1.data = msgs_data.at(0U);
    msg2.data = msgs_data.at(1U);
    msg3.data = msgs_data.at(2U);
    publisher_functions.emplace_back(([this, msg1]() {pub1_->publish(msg1);}));
    publisher_functions.emplace_back(([this, msg2]() {pub2_->publish(msg2);}));
    publisher_functions.emplace_back(([this, msg3]() {pub3_->publish(msg3);}));

    while (rclcpp::ok()) {
      std::shuffle(publish_order.begin(), publish_order.end(), e);
      RCLCPP_INFO(
        this->get_logger(), "Publishing: %s%s%s",
        msgs_data[publish_order.at(0)].c_str(),
        msgs_data[publish_order.at(1)].c_str(),
        msgs_data[publish_order.at(2)].c_str());
      publisher_functions.at(publish_order.at(0))();
      publisher_functions.at(publish_order.at(1))();
      publisher_functions.at(publish_order.at(2))();
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
  auto print_msg = [&node](std_msgs::msg::String::UniquePtr msg) {
    RCLCPP_INFO(node->get_logger(), "I heard: %s", msg->data.c_str());
  };

  auto sub1 = node->create_subscription<std_msgs::msg::String>("topicA", 10, print_msg);
  auto sub2 = node->create_subscription<std_msgs::msg::String>("topicB", 10, print_msg);
  auto sub3 = node->create_subscription<std_msgs::msg::String>("topicC", 10, print_msg);

  rclcpp::WaitSet wait_set({{sub1}, {sub2}, {sub3}});

  // Creates a random publisher and starts publishing in another thread
  RandomPublisher publisher_node;
  auto publisher_thread = std::thread([&publisher_node]() {publisher_node.run();});

  while (rclcpp::ok()) {
    // Waiting up to 5s for a message to arrive
    const auto wait_result = wait_set.wait(std::chrono::seconds(5));

    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      bool sub1_has_data = wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U];
      bool sub2_has_data = wait_result.get_wait_set().get_rcl_wait_set().subscriptions[1U];
      bool sub3_has_data = wait_result.get_wait_set().get_rcl_wait_set().subscriptions[2U];

      // We will handle all the messages when all subscriptions have received data.
      // Note we could use just one subscription or a combination of conditions to trigger data
      // handling.
      bool handle_condition = sub1_has_data && sub2_has_data && sub3_has_data;

      if (handle_condition) {
        std_msgs::msg::String msg1, msg2, msg3;
        rclcpp::MessageInfo msg_info;

        // the messages are taken and handled in a user-defined order
        if (sub1->take(msg1, msg_info)) {
          std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg1);
          sub1->handle_message(type_erased_msg, msg_info);
        }
        if (sub2->take(msg2, msg_info)) {
          std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg2);
          sub2->handle_message(type_erased_msg, msg_info);
        }
        if (sub3->take(msg3, msg_info)) {
          std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg3);
          sub3->handle_message(type_erased_msg, msg_info);
        }
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      RCLCPP_ERROR(node->get_logger(), "No message received after 5s.");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Wait-set failed.");
    }
  }

  rclcpp::shutdown();
  publisher_thread.join();
  return 0;
}
