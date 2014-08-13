#include <iostream>
#include <sys/time.h>
#include <chrono>
#include <thread>

#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"
#include "simple_msgs/Int32.h"


int main(int argc, char** argv)
{
  rclcpp::Node* n = rclcpp::create_node();
  rclcpp::Publisher<simple_msgs::Int32>* p = n->create_publisher<simple_msgs::Int32>("topic_name");
  simple_msgs::Int32 ros_msg;

  int number_of_msgs = 1000000;

  auto start = std::chrono::steady_clock::now();
  for (int i = 1; i < number_of_msgs; ++i) {
    ros_msg.data = i;
    p->publish(ros_msg);
    // if (i % 100000 == 0) {
      std::cout << "published ros msg #" << i << std::endl;
    // }
    std::chrono::milliseconds dura(2000);
    std::this_thread::sleep_for(dura);
  }
  auto end = std::chrono::steady_clock::now();

  std::cout << (end - start).count() << std::endl;

  return 0;
}
