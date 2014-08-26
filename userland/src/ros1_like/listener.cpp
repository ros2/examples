#include <iostream>
#include <memory>

// #include "ros/ros.h"
#include <rclcpp/rclcpp.hpp>

// #include "std_msgs/String.h"
#include <simple_msgs/String.h>

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
void chatterCallback(const simple_msgs::String::ConstPtr &msg)
{
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
  std::cout << "I heard: [" << msg->data << "]" << std::endl;
  // std::cout << "I heard: [" << msg.data << "]" << std::endl;
}

int main(int argc, char *argv[])
{
  // ros::init(argc, argv, "listener");
  rclcpp::init(argc, argv);
  // ros::NodeHandle n;
  auto node = rclcpp::Node::make_shared("listener");

  // ros::Subscriber sub = n.subscribe("chatter", 7, chatterCallback);
  auto sub = node->create_subscription<simple_msgs::String>("chatter", 7, chatterCallback);

  // ros::spin();
  rclcpp::spin(node);

  return 0;
}
