#include <chrono>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class SingleThreadSubscriber : public rclcpp::Node {

public:
  SingleThreadSubscriber() : Node("subscriber_node") {
    short_subscriber_ = create_subscription<std_msgs::msg::String>(
        "/short_topic", rclcpp::QoS(10),
        std::bind(&SingleThreadSubscriber::ShortTopicCallback, this,
                  std::placeholders::_1));

    long_subscriber_ = create_subscription<std_msgs::msg::String>(
        "/long_topic", rclcpp::QoS(10),
        std::bind(&SingleThreadSubscriber::LongTopicCallback, this,
                  std::placeholders::_1));
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
SingleThreadSubscriber::ProcessString(const std::string &raw_string) {
  std::this_thread::sleep_for(std::chrono::seconds(2));
  return "** " + raw_string + " **";
};

void SingleThreadSubscriber::ShortTopicCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  auto processed_string = ProcessString(msg->data);

  RCLCPP_INFO(get_logger(), "Setting processed:  %s", processed_string.c_str());
  processed_short_string_ = processed_string;
}

void SingleThreadSubscriber::LongTopicCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  auto processed_string = ProcessString(msg->data);

  RCLCPP_INFO(get_logger(), "Setting processed:  %s", processed_string.c_str());
  processed_long_string_ = processed_string;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SingleThreadSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
