#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax slightly. */

struct Talker : public rclcpp::Node
{
  Talker() : Node("talker"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter");
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(this->count_++);
      printf("Publishing: [%s]\n", message.data.c_str());
      this->publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(500_ms, timer_callback);
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  return 0;
}
