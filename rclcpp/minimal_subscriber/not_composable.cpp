// Copyright 2016 Open Source Robotics Foundation, Inc.
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

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("minimal_subscriber"), "I heard: '%s'", msg->data.c_str())
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // TODO(clalancette): We can't currently make g_node a global because of
  // https://github.com/eProsima/Fast-RTPS/issues/235 .  Once that is fixed
  // we may want to add another example that uses global nodes.
  auto node = rclcpp::Node::make_shared("minimal_subscriber");
  auto subscription = node->create_subscription<std_msgs::msg::String>
      ("topic", topic_callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
