#include <random>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node {
public:
  PublisherNode() : Node("publisher_node") {
    short_publisher_ = create_publisher<std_msgs::msg::String>(
        "/short_topic", rclcpp::SystemDefaultsQoS());
    long_publisher_ = create_publisher<std_msgs::msg::String>(
        "/long_topic", rclcpp::SystemDefaultsQoS());

    short_timer_ =
        create_wall_timer(100ms, [this]() { PublishShortMessage(); });

    long_timer_ = create_wall_timer(1s, [this]() { PublishLongMessage(); });
  }

private:
  size_t short_seq_{0};
  size_t long_seq_{0};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr short_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr long_publisher_;

  rclcpp::TimerBase::SharedPtr short_timer_;
  rclcpp::TimerBase::SharedPtr long_timer_;

  void PublishShortMessage() {
    std_msgs::msg::String message;
    message.data = "Short message (seq=" + std::to_string(short_seq_++) + ")";
    RCLCPP_INFO(get_logger(), message.data, " published.");
    short_publisher_->publish(message);
  }

  void PublishLongMessage() {
    std_msgs::msg::String message;
    message.data = "Long message with more characters (seq=" +
                   std::to_string(long_seq_++) + ")";
    RCLCPP_INFO(get_logger(), message.data, " published.");
    long_publisher_->publish(message);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
