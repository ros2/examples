/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void chatter_callback(const std_msgs::msg::String::SharedPtr msg)
{
  printf("I heard: [%s]\n", msg->data.c_str());
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("old_school_listener");
  auto subscription = node->create_subscription<std_msgs::msg::String>
      ("chatter", chatter_callback);
  rclcpp::spin(node);
  return 0;
}
