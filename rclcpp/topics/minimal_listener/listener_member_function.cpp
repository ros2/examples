#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

struct Listener : public rclcpp::Node
{
  Listener() : Node("listener")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>
        ("chatter", std::bind(&Listener::chatter_callback, this, _1));
  }
private:
  void chatter_callback(const std_msgs::msg::String::SharedPtr msg) {
    printf("I heard: [%s]\n", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  return 0;
}
