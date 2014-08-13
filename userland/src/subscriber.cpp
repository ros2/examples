#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"
#include "rclcpp/executor/SingleThreadExecutor.h"
#include "simple_msgs/Int32.h"

int main(int argc, char** argv)
{
  rclcpp::Node *node = rclcpp::create_node();
  rclcpp::Subscriber<simple_msgs::Int32> *subscriber = node->create_subscriber<simple_msgs::Int32>("topic_name");

  rclcpp::executor::SingleThreadExecutor *executor = new rclcpp::executor::SingleThreadExecutor();

  executor->register_node(node);
  executor->exec();

  return 0;
}
