#include <chrono>
#include <functional>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/Uint32.h>


template<typename T>
int publish(
  rclcpp::Node::SharedPtr node,
  std::function<void(typename T::Ptr&, size_t)> set_data_func,
  size_t number_of_messages)
{
  auto p = node->create_publisher<T>("topic_name", 10);
  typename T::Ptr ros_msg(new T());

  auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(1);
  size_t i = 1;
  while (rclcpp::ok() and i <= number_of_messages) {
    set_data_func(ros_msg, i);
    p->publish(ros_msg);
    std::cout << "published ros msg #" << i << std::endl;
    ++i;
    rate.sleep();
  }
  auto end = std::chrono::steady_clock::now();

  std::chrono::duration<float> diff = (end - start);
  std::cout << "Runtime: " << diff.count() << " seconds" << std::endl;

  return 0;
}

void set_counter_data(simple_msgs::Uint32::Ptr &ros_msg, size_t i)
{
  ros_msg->data = i;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_publisher");

  return publish<simple_msgs::Uint32>(node, &set_counter_data, 10);
}
