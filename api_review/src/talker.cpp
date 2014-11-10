#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <simple_msgs/String.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::node::NodeHandle> nh = rclcpp::node::create_node("talker");
  std::unique_ptr<rclcpp::publisher::PublisherHandle> pub = \
    rclcpp::publisher::create_publisher<simple_msgs::String>(nh, "/chatter", 10);
  return 0;
}
