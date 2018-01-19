// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalRemappingSubscriber : public rclcpp::Node
{
public:
  MinimalRemappingSubscriber()
  : Node("minimal_remapping_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/foo/bar",
      [this](std_msgs::msg::String::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str())
    });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int /*argc*/, char * argv[])
{
  const char * argForRemap[2] = {argv[0], "/foo/bar:=/bar/foo"};
  rclcpp::init(2, const_cast<char **>(argForRemap));
  rclcpp::spin(std::make_shared<MinimalRemappingSubscriber>());
  rclcpp::shutdown();
  return 0;
}
