#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/Uint32.h>


template<typename T>
rclcpp::subscription::SubscriptionBase::SharedPtr subscribe(rclcpp::Node::SharedPtr node, typename rclcpp::subscription::Subscription<T>::CallbackType callback)
{
  auto sub = node->create_subscription<T>("topic_name", 10, callback);
  return sub;
}

void print_counter_data(const simple_msgs::Uint32::ConstPtr &msg)
{
  std::cout << "Got message #" << msg->data << std::endl;
  rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_subscriber");
  auto sub = subscribe<simple_msgs::Uint32>(node, print_counter_data);
  rclcpp::spin(node);
  return 0;
}
