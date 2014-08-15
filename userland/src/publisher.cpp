#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/time.h>
#include <thread>

#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"

#include "simple_msgs/AllPrimitiveTypes.h"
#include "simple_msgs/Uint32.h"

#include "userland/command_line_arguments.h"


template<typename T>
int publish(rclcpp::Node* node, void (*set_data_func)(T&, uint32_t))
{
  rclcpp::Publisher<T>* p = node->create_publisher<T>("topic_name");
  T ros_msg;

  //int number_of_msgs = 1000000;

  auto start = std::chrono::steady_clock::now();
  uint32_t i = 1;
  while (true) {
    set_data_func(ros_msg, i);
    p->publish(ros_msg);
    // if (i % 100000 == 0) {
      std::cout << "published ros msg #" << i << std::endl;
    // }
    ++i;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  auto end = std::chrono::steady_clock::now();

  std::cout << (end - start).count() << std::endl;

  return 0;
}

void set_counter_data(simple_msgs::Uint32& ros_msg, uint32_t i)
{
  ros_msg.data = i;
}

void set_all_primitive_data(simple_msgs::AllPrimitiveTypes& ros_msg, uint32_t i)
{
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
}

int main(int argc, char** argv)
{
  if (has_argument(argv, argv + argc, "--help")) {
    std::cout << "usage: " << argv[0] << std::endl;
    print_message_usage();
    return 0;
  }

  rclcpp::Node* node = rclcpp::create_node();

  const std::string msg_arg = get_named_argument(argv, argv + argc, "--msg", valid_message_args[0]);
  if (msg_arg == valid_message_args[0]) {
    return publish<simple_msgs::Uint32>(node, &set_counter_data);
  } else if (msg_arg == valid_message_args[1]) {
    return publish<simple_msgs::AllPrimitiveTypes>(node, &set_all_primitive_data);
  }

  std::cerr << "unsupported '--msg' argument '" << msg_arg << "'" << std::endl;
  return 1;
}
