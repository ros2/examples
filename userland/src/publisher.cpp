// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/AllBuiltinTypes.h>
#include <simple_msgs/AllDynamicArrayTypes.h>
#include <simple_msgs/AllPrimitiveTypes.h>
#include <simple_msgs/AllStaticArrayTypes.h>
#include <simple_msgs/Nested.h>
#include <simple_msgs/String.h>
#include <simple_msgs/Uint32.h>

#include "userland/command_line_arguments.h"


template<typename T>
int publish(rclcpp::Node::SharedPtr node, void (*set_data_func)(typename T::Ptr&, size_t))
{
  auto p = node->create_publisher<T>("topic_name", 1000);
  typename T::Ptr ros_msg(new T());

  auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(10);
  size_t i = 1;
  while (rclcpp::ok()) {
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

void set_all_primitive_data(simple_msgs::AllPrimitiveTypes::Ptr &ros_msg, size_t i)
{
  ros_msg->my_bool = i % 2;
  ros_msg->my_byte = i % 256;
  ros_msg->my_char = i % 256;
  ros_msg->my_float32 = 0.1f + i;
  ros_msg->my_float64 = 0.1 + i;
  ros_msg->my_int8 = i;
  ros_msg->my_uint8 = i;
  ros_msg->my_int16 = i;
  ros_msg->my_uint16 = i;
  ros_msg->my_int32 = i;
  ros_msg->my_uint32 = i;
  ros_msg->my_int64 = i;
  ros_msg->my_uint64 = i;
  ros_msg->my_string = "foo " + std::to_string(i);
}

void set_all_static_array(simple_msgs::AllStaticArrayTypes::Ptr &ros_msg, size_t i)
{
  int start = i - 1; // get the zero
  int end = i + 5; // assuming that the array size is 6
  int iteration_step;
  for (int j = start; j < end; ++j){
    iteration_step = j - i + 1;
    ros_msg->my_bool[iteration_step] = j % 2;
    ros_msg->my_byte[iteration_step] = j % 256;
    ros_msg->my_char[iteration_step] = j % 256;
    ros_msg->my_float32[iteration_step] = 0.1f + j;
    ros_msg->my_float64[iteration_step] = 0.1 + j;
    ros_msg->my_int8[iteration_step] = j;
    ros_msg->my_uint8[iteration_step] = j;
    ros_msg->my_int16[iteration_step] = j;
    ros_msg->my_uint16[iteration_step] = j;
    ros_msg->my_int32[iteration_step] = j;
    ros_msg->my_uint32[iteration_step] = j;
    ros_msg->my_int64[iteration_step] = j;
    ros_msg->my_uint64[iteration_step] = j;
    ros_msg->my_string[iteration_step] = std::string("foo ") + std::to_string(j);
  }
}

void set_all_dynamic_array(simple_msgs::AllDynamicArrayTypes::Ptr &ros_msg, size_t i)
{
  int array_size = i - 1;
  ros_msg->my_bool.resize(array_size);
  ros_msg->my_byte.resize(array_size);
  ros_msg->my_char.resize(array_size);
  ros_msg->my_float32.resize(array_size);
  ros_msg->my_float64.resize(array_size);
  ros_msg->my_int8.resize(array_size);
  ros_msg->my_uint8.resize(array_size);
  ros_msg->my_int16.resize(array_size);
  ros_msg->my_uint16.resize(array_size);
  ros_msg->my_int32.resize(array_size);
  ros_msg->my_uint32.resize(array_size);
  ros_msg->my_int64.resize(array_size);
  ros_msg->my_uint64.resize(array_size);
  ros_msg->my_string.resize(array_size);

  for (int j = array_size; j < 2 * array_size; ++j){
    ros_msg->my_bool[j - array_size] = j % 2;
    ros_msg->my_byte[j - array_size] = j % 256;
    ros_msg->my_char[j - array_size] = j % 256;
    ros_msg->my_float32[j - array_size] = 0.1f + j;
    ros_msg->my_float64[j - array_size] = 0.1 + j;
    ros_msg->my_int8[j - array_size] = j;
    ros_msg->my_uint8[j - array_size] = j;
    ros_msg->my_int16[j - array_size] = j;
    ros_msg->my_uint16[j - array_size] = j;
    ros_msg->my_int32[j - array_size] = j;
    ros_msg->my_uint32[j - array_size] = j;
    ros_msg->my_int64[j - array_size] = j;
    ros_msg->my_uint64[j - array_size] = j;
    ros_msg->my_string[j - array_size] = std::string("foo ") + std::to_string(j);
  }
}

void set_nested(simple_msgs::Nested::Ptr &ros_msg, size_t i)
{
  ros_msg->submsg.data = i;
}

void set_string(simple_msgs::String::Ptr &ros_msg, size_t i)
{
  i = std::pow(2, i) - 1;
  ros_msg->data = "";
  for (uint64_t j = i; j < 2 * i; ++j) {
    ros_msg->data += std::to_string(j % 10);
  }
}

template<typename T>
void set_empty(typename T::Ptr &ros_msg, size_t i)
{
  ros_msg.reset(new T());
}

void set_builtin(simple_msgs::AllBuiltinTypes::Ptr &ros_msg, size_t i)
{
  ros_msg->my_duration.sec = -i;
  ros_msg->my_duration.nanosec = i;
  ros_msg->my_time.sec = - 2 * i;
  ros_msg->my_time.nanosec = 2 * i;
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

  const std::string msg_arg = get_named_argument(argv, argv + argc, "--msg", valid_message_args[0]);
  if (msg_arg == valid_message_args[0]) {
    return publish<simple_msgs::Uint32>(node, &set_counter_data);
  } else if (msg_arg == valid_message_args[1]) {
    return publish<simple_msgs::AllPrimitiveTypes>(node, &set_all_primitive_data);
  } else if (msg_arg == valid_message_args[2]) {
    return publish<simple_msgs::AllStaticArrayTypes>(node, &set_all_static_array);
  } else if (msg_arg == valid_message_args[3]) {
    return publish<simple_msgs::AllDynamicArrayTypes>(node, &set_all_dynamic_array);
  } else if (msg_arg == valid_message_args[4]) {
    return publish<simple_msgs::Nested>(node, &set_nested);
  } else if (msg_arg == valid_message_args[5]) {
    return publish<simple_msgs::String>(node, &set_string);
  } else if (msg_arg == valid_message_args[6]) {
    return publish<simple_msgs::AllPrimitiveTypes>(node, &set_empty<simple_msgs::AllPrimitiveTypes>);
  } else if (msg_arg == valid_message_args[7]) {
    return publish<simple_msgs::AllBuiltinTypes>(node, &set_builtin);
  }


  std::cerr << "unsupported '--msg' argument '" << msg_arg << "'" << std::endl;
  return 1;
}
