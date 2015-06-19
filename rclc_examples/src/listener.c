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

#include <stdio.h>

#include <rclc/rclc.h>

#include <std_interfaces/msg/Int32.h>

typedef std_interfaces__msg__Int32 Int32;

void callback(const void * void_message)
{
  const Int32 * message = (const Int32 *)void_message;
  printf("Got number: %d\n", message->data);
}

int main(int argc, char ** argv)
{
  rclc_init(argc, argv);

  rclc_node_t * node = rclc_create_node("listener");

  rclc_subscription_t * subscription = rclc_create_subscription(
    node, ROSIDL_GET_MESSAGE_TYPE_SUPPORT(std_interfaces, Int32), "chatter", callback, 7);

  rclc_spin_node(node);

  rclc_destroy_subscription(subscription);
  rclc_destroy_node(node);

  return 0;
}
