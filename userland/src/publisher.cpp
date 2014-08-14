#include <iostream>
#include <sys/time.h>
#include <chrono>
#include <thread>

#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"
#include "simple_msgs/AllPrimitiveTypes.h"


int main(int argc, char** argv)
{
  rclcpp::Node* n = rclcpp::create_node();
  rclcpp::Publisher<simple_msgs::AllPrimitiveTypes>* p = n->create_publisher<simple_msgs::AllPrimitiveTypes>("topic_name");
  simple_msgs::AllPrimitiveTypes ros_msg;

  int number_of_msgs = 1000000;

  auto start = std::chrono::steady_clock::now();
  for (int i = 1; i < number_of_msgs; ++i) {
    ros_msg.my_bool = i % 2;
    ros_msg.my_byte = i % 256;
    ros_msg.my_char = i % 256;
    ros_msg.my_float32 = 0.1f + i;
    ros_msg.my_float64 = 0.1 + i;
    ros_msg.my_int8 = i;
    ros_msg.my_uint8 = i;
    ros_msg.my_int16 = i;
    ros_msg.my_uint16 = i;
    ros_msg.my_int32 = i;
    ros_msg.my_uint32 = i;
    ros_msg.my_int64 = i;
    ros_msg.my_uint64 = i;
    ros_msg.my_string = "foo " + std::to_string(i);
    p->publish(ros_msg);
    // if (i % 100000 == 0) {
      std::cout << "published ros msg #" << i << std::endl;
    // }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  auto end = std::chrono::steady_clock::now();

  std::cout << (end - start).count() << std::endl;

  return 0;
}
