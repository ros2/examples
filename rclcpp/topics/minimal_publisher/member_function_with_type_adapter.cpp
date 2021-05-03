// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* Normally a TypeAdapter specialization like this would go in a header
 * and be reused by the publisher and subscriber rather than copy-pasted
 * like this. We chose to include this here because it makes this example
 * more "self-contained". */

template<>
struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
{
  using is_specialized = std::true_type;
  using custom_type = std::string;
  using ros_message_type = std_msgs::msg::String;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.data = source;
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = source.data;
  }
};

/* In this example, a publisher uses a type adapter to use a `std::string`
 * in place of a `std_msgs::msg::String` in the argument expected by
 * the publish method. Note that publish will also work with a
 * `std_msgs::msg::String` argument. */

class MinimalPublisher : public rclcpp::Node
{
  using MyAdaptedType = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;

public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<MyAdaptedType>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std::string message = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<MyAdaptedType>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
