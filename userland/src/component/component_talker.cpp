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

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/msg/string.hpp>

namespace userland
{
class RMW_EXPORT Talker : public Node
{
public:
  Talker(rclcpp::ContextSharedPtr context)
  : Node("talker", context), count_(0)
  {
    chatter_pub_ = this->create_publisher<simple_msgs::msg::String>("chatter", 7);
    publish_timer_ = this->create_wall_timer(0.5_s, std::bind(&Talker::on_timer, this));
  }

  void on_timer()
  {
    auto msg = std::make_shared<simple_msgs::msg::String>();
    msg->data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO("Publishing: '" << msg->data << "'");
    chatter_pub_->publish(msg);
  }

private:
  size_t count_;
  rclcpp::Publisher::SharedPtr chatter_pub_;
  rclcpp::WallTimer::SharedPtr publish_timer_;

};
}

RCLCPP_REGISTER_NODE(userland::Talker);
