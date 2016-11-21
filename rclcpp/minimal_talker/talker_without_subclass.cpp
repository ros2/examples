#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("old_school_talker");
  auto publisher = node->create_publisher<std_msgs::msg::String>("chatter");
  auto message = std::make_shared<std_msgs::msg::String>();
  auto publish_count = 0;
  while (rclcpp::ok()) {
    message->data = "Hello, world! " + std::to_string(publish_count++);
    printf("Publishing: [%s]\n", message->data.c_str());
    publisher->publish(message);
    rclcpp::spin_some(node);
    rclcpp::utilities::sleep_for(std::chrono::milliseconds(500));
  }
  return 0;
}
