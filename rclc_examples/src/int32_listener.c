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

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/int32.h>

typedef std_msgs__msg__Int32 Int32;

void fail_if_not_ok(rcl_ret_t ret)
{
  if (ret != RCL_RET_OK) {
    fprintf(stderr, "Error: %s\n", rcl_get_error_string_safe());
    exit(1);
  }
}

void callback(const void * void_message)
{
  const Int32 * message = (const Int32 *)void_message;
  printf("Got number: %d\n", message->data);
}

int main(int argc, char ** argv)
{
  rcl_ret_t ret = rcl_init(argc, argv, rcl_get_default_allocator());
  fail_if_not_ok(ret);

  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_options_t default_node_options = rcl_node_get_default_options();
  ret = rcl_node_init(&node, "listener", &default_node_options);
  fail_if_not_ok(ret);

  rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t default_subscription_options = rcl_subscription_get_default_options();
  ret = rcl_subscription_init(
    &subscription, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, Int32), "chatter",
    &default_subscription_options);
  fail_if_not_ok(ret);

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(&wait_set, 1, 0, 0, rcl_get_default_allocator());
  fail_if_not_ok(ret);
  while (1) {
    ret = rcl_wait_set_clear_subscriptions(&wait_set);
    fail_if_not_ok(ret);
    ret = rcl_wait_set_add_subscription(&wait_set, &subscription);
    fail_if_not_ok(ret);
    ret = rcl_wait(&wait_set, RCL_S_TO_NS(1));
    if (ret == RCL_RET_TIMEOUT) {
      continue;
    }
    fail_if_not_ok(ret);
    bool taken = false;
    Int32 taken_msg;
    ret = rcl_take(&subscription, &taken_msg, &taken, NULL);
    fail_if_not_ok(ret);
    if (taken) {
      callback(&taken_msg);
    }
  }
  ret = rcl_wait_set_fini(&wait_set);
  fail_if_not_ok(ret);

  ret = rcl_subscription_fini(&subscription, &node);
  fail_if_not_ok(ret);
  ret = rcl_node_fini(&node);
  fail_if_not_ok(ret);

  return 0;
}
