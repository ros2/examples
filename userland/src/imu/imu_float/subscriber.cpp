#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simple_msgs/Vector3Float.h>
#include "userland/command_line_arguments.h"


template<typename T>
rclcpp::subscription::SubscriptionBase::SharedPtr subscribe(rclcpp::Node::SharedPtr node, typename rclcpp::subscription::Subscription<T>::CallbackType callback)
{
  auto sub = node->create_subscription<T>("imu", 1000, callback);
  return sub;
}

void print_accel_data(const simple_msgs::Vector3Float::ConstPtr &msg)
{
  std::cout << "-------------------------"<< std::endl;
  std::cout << "Got accel x=" << msg->x << std::endl;
  std::cout << "Got accel y=" << msg->y << std::endl;
  std::cout << "Got accel z=" << msg->z << std::endl;
  std::cout << "-------------------------"<< std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  if (has_argument(argv, argv + argc, "--help")) {
    std::cout << "usage:" << argv[0] << std::endl;
    print_message_usage();
    return 0;
  }

  auto node = rclcpp::Node::make_shared("subscriber");
  rclcpp::subscription::SubscriptionBase::SharedPtr sub;

  sub = subscribe<simple_msgs::Vector3Float>(node, print_accel_data);
  rclcpp::spin(node);

  return 0;
}
