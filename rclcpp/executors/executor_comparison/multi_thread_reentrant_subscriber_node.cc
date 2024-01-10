#include <chrono>
#include <random>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class MultiThreadReentrantSubscriber : public rclcpp::Node {

public:
  MultiThreadReentrantSubscriber() : Node("subscriber_node") {
    rclcpp::SubscriptionOptions options;
    options.callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    short_subscriber_ = create_subscription<std_msgs::msg::String>(
        "/short_topic", rclcpp::QoS(10),
        std::bind(&MultiThreadReentrantSubscriber::ShortTopicCallback, this,
                  std::placeholders::_1),
        options);

    long_subscriber_ = create_subscription<std_msgs::msg::String>(
        "/long_topic", rclcpp::QoS(10),
        std::bind(&MultiThreadReentrantSubscriber::LongTopicCallback, this,
                  std::placeholders::_1),
        options);
  }

private:
  std::string processed_short_string_;
  std::string processed_long_string_;
  std::string ProcessString(const std::string &raw_string);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr short_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr long_subscriber_;
  void ShortTopicCallback(const std_msgs::msg::String::SharedPtr msg);
  void LongTopicCallback(const std_msgs::msg::String::SharedPtr msg);
};

std::string
MultiThreadReentrantSubscriber::ProcessString(const std::string &raw_string) {
  std::thread::id this_id = std::this_thread::get_id();
  std::ostringstream oss;
  oss << std::this_thread::get_id();

  std::this_thread::sleep_for(std::chrono::seconds(2));
  return "** " + raw_string + " **" + " ( by " + oss.str() + ") ";
};

void MultiThreadReentrantSubscriber::ShortTopicCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  auto processed_string = ProcessString(msg->data);

  RCLCPP_INFO(get_logger(), "Setting processed:  %s", processed_string.c_str());
  processed_short_string_ = processed_string;
}

void MultiThreadReentrantSubscriber::LongTopicCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  auto processed_string = ProcessString(msg->data);

  RCLCPP_INFO(get_logger(), "Setting processed:  %s", processed_string.c_str());
  processed_long_string_ = processed_string;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiThreadReentrantSubscriber>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
