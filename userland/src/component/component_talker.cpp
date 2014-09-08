#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/String.h>

namespace userland {
class Talker : public Node
{
public:
  Talker(rclcpp::ContextSharedPtr context) : Node("talker", context), count_(0)
  {
    chatter_pub_ = this->create_publisher<simple_msgs::String>("chatter", 7);
    publish_timer_ = this->create_wall_timer(0.5_s, std::bind(&Talker::on_timer, this));
  }

  void on_timer()
  {
    auto msg = std::make_shared<simple_msgs::String>();
    msg->data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO("Publishing: '" << msg->data << "'");
    chatter_pub_->publish(msg);
  }

private:
  size_t count_;
  rclcpp::Publisher::SharedPtr chatter_pub_;
  rclcpp::WallTimer::SharedPtr publish_timer_;

};
}

RCLCPP_REGISTER_NODE(userland::Talker);
