/* Copyright 2014 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RCL_RCL_RCL_H_
#define RCL_RCL_RCL_H_

#include <stdbool.h>

#include <rmw/rmw.h>

#include "types.h"

rcl_ret_t
rcl_init(int argc, char **argv);

rcl_node_t *
rcl_create_node(const char * name);

rcl_ret_t
rcl_destroy_node();

rcl_publisher_t *
rcl_create_publisher(const rcl_node_t * node,
                     const rosidl_message_type_support_t * type_support,
                     const char * topic_name);

rcl_ret_t
rcl_destroy_publisher(rcl_publisher_t * publisher);

// TODO: How do we allow for void * to be cast to CPP type or C type
//       depending on the user?
rcl_ret_t
rcl_publish(const rcl_publisher_t * publisher, const void * ros_message);

rcl_subscription_t *
rcl_create_subscription(const rcl_node_t * node,
                        const rosidl_message_type_support_t * type_support,
                        const char * topic_name);

rcl_ret_t
rcl_destroy_subscription(rcl_subscription_t * subscription);

// TODO: How do we allow for void * to be cast to CPP type or C type
//       depending on the user?
rcl_ret_t
rcl_take(const rcl_subscription_t * subscriber, void * ros_message);

rcl_guard_condition_t *
rcl_create_guard_condition();

rcl_ret_t
rcl_destroy_guard_condition(rcl_guard_condition_t * guard_condition);

rcl_ret_t
rcl_trigger_guard_condition(const rcl_guard_condition_t * guard_condition);

rcl_callback_group_t *
rcl_create_callback_group(rcl_node_t * node);

rcl_ret_t
rcl_destroy_callback_group(rcl_callback_group_t * callback_group);

rcl_subscription_info_t *
rcl_create_subscription_info(rcl_subscription_t * const subscription,
                             rcl_callback_group_t * const callback_group);

rcl_ret_t
rcl_destroy_subscription_info(rcl_subscription_info_t * subscription_info);

rcl_timer_info_t *
rcl_create_timer_info(rcl_guard_condition_t * const guard_condition,
                      rcl_callback_group_t * const callback_group);

rcl_ret_t
rcl_destroy_timer_info(rcl_timer_info_t * timer_info);

rcl_executor_helper_t *
rcl_create_executor_helper();

rcl_ret_t
rcl_destroy_executor_helper(rcl_executor_helper_t * executor_helper);

rcl_ret_t
rcl_add_subscription_info(rcl_executor_helper_t * executor_helper,
                          rcl_subscription_info_t * subscription_info);

rcl_ret_t
rcl_remove_subscription_info(rcl_executor_helper_t * executor_helper,
                             rcl_subscription_info_t * subscription_info);

rcl_ret_t
rcl_get_next_any_executable(rcl_executor_helper_t * executor_helper,
                            rcl_any_executable_t * any_executable,
                            bool non_blocking);

#endif  /* RCL_RCL_RCL_H_ */
