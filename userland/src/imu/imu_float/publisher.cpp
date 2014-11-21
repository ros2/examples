#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <sys/time.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <simple_msgs/Vector3Float.h>
#include "userland/command_line_arguments.h"


template<typename T>
int publish(rclcpp::Node::SharedPtr node, void (*set_data_func)(typename T::Ptr&, size_t))
{
  auto p = node->create_publisher<T>("imu", 1000);
  typename T::Ptr ros_msg(new T());

  auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(10);
  size_t i = 1;
  while (rclcpp::ok()) {
    set_data_func(ros_msg, i);
    p->publish(ros_msg);
      std::cout << "published Vector3Float ROS msg #" << i << std::endl;
    ++i;
    rate.sleep();
  }
  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<float> diff = (end - start);
  std::cout << "Runtime: " << diff.count() << " seconds" << std::endl;

  return 0;
}

void set_accel_data(simple_msgs::Vector3Float::Ptr &ros_msg, size_t i)
{
  ros_msg->x = 1;
  ros_msg->y = 2;
  ros_msg->z = 3;  
}

template<typename T>
void set_empty(typename T::Ptr &ros_msg, size_t i)
{
  ros_msg.reset(new T());
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  if (has_argument(argv, argv + argc, "--help")) {
    std::cout << "usage: " << argv[0] << std::endl;
    print_message_usage();
    return 0;
  }

  auto node = rclcpp::Node::make_shared("publisher");
  return publish<simple_msgs::Vector3Float>(node, &set_accel_data);
}
