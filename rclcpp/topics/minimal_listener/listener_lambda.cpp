#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

struct Listener : public rclcpp::Node
{
  Listener() : Node("listener")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>("chatter",
        [](std_msgs::msg::String::UniquePtr msg) {
          printf("I heard: [%s]\n", msg->data.c_str());
        });
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  return 0;
}
