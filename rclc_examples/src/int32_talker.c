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

#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/Int32.h>

void fail_if_not_ok(rcl_ret_t ret)
{
  if (ret != RCL_RET_OK) {
    fprintf(stderr, "Error: %s\n", rcl_get_error_string_safe());
    exit(1);
  }
}

int main(int argc, char ** argv)
{
  rcl_ret_t ret = rcl_init(argc, argv, rcl_get_default_allocator());
  fail_if_not_ok(ret);

  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_options_t default_node_options = rcl_node_get_default_options();
  ret = rcl_node_init(&node, "talker", &default_node_options);
  fail_if_not_ok(ret);

  rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t default_publisher_options = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, Int32), "chatter",
    &default_publisher_options);
  fail_if_not_ok(ret);

  size_t count = 0;
  while (rcl_ok()) {
    std_msgs__msg__Int32 msg;
    msg.data = count++;
    printf("Publishing %d\n", msg.data);
    ret = rcl_publish(&publisher, &msg);
    fail_if_not_ok(ret);
    // TODO(wjwwood): make a portable sleep in rcl? Maybe it should only be in rclc?
    sleep(1);
  }

  ret = rcl_publisher_fini(&publisher, &node);
  fail_if_not_ok(ret);
  ret = rcl_node_fini(&node);
  fail_if_not_ok(ret);

  return 0;
}
