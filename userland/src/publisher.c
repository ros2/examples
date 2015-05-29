// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <rclc/rclc.h>

#include <simple_msgs/msg/Int32-c.h>

int main(int argc, char ** argv)
{
  rclc_init(argc, argv);

  rclc_node_t * node = rclc_create_node("publisher");

  rclc_publisher_t * publisher = rclc_create_publisher(
    node, ROSIDL_GET_MESSAGE_TYPE_SUPPORT(simple_msgs, Int32), "chatter", 7);

  size_t count = 0;
  while (rclc_ok()) {
    simple_msgs__Int32 msg;
    msg.data = count++;
    rclc_publish(publisher, &msg);
    rclc_sleep_ms(1000);
  }

  rclc_destroy_publisher(publisher);
  rclc_destroy_node(node);

  return 0;
}
