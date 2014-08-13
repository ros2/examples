#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"
#include "rclcpp/executor/SingleThreadExecutor.h"
#include "simple_msgs/Int32.h"

void callback(const simple_msgs::Int32 *msg)
{
    std::cout << "Got message: " << msg->data << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::Node *node = rclcpp::create_node();
  node->create_subscriber<simple_msgs::Int32>("topic_name", callback);

  rclcpp::executor::SingleThreadExecutor *executor = new rclcpp::executor::SingleThreadExecutor();

  executor->register_node(node);
  executor->exec();

  return 0;
}
