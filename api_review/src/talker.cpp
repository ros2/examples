#include <memory>

#include <api_review/rclcpp/rclcpp.hpp>
#include <api_review/rclcpp/node.hpp>
#include <api_review/rclcpp/publisher.hpp>

#include <simple_msgs/String.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::node::NodeHandle> nh = rclcpp::node::create_node("talker");
  std::unique_ptr<rclcpp::publisher::PublisherHandle> pub = \
    rclcpp::publisher::create_publisher<simple_msgs::String>(nh, "/chatter", 10);
  return 0;
}
