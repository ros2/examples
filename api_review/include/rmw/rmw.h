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

#ifndef RMW_RMW_RMW_H_
#define RMW_RMW_RMW_H_

#include <stdbool.h>

#include <rosidl/message_type_support.h>

#include "types.h"

rmw_ret_t
rmw_init();

rmw_node_t *
rmw_create_node(const char * name);

rmw_ret_t
rmw_destroy_node();

rmw_publisher_t *
rmw_create_publisher(const rmw_node_t * node,
                     const rosidl_message_type_support_t * type_support,
                     const char * topic_name);

rmw_ret_t
rmw_destroy_publisher(rmw_publisher_t * publisher);

// TODO: How do we allow for void * to be cast to CPP type or C type
//       depending on the user?
rmw_ret_t
rmw_publish(const rmw_publisher_t * publisher, const void * ros_message);

rmw_subscription_t *
rmw_create_subscription(const rmw_node_t * node,
                        const rosidl_message_type_support_t * type_support,
                        const char * topic_name);

rmw_ret_t
rmw_destroy_subscription(rmw_subscription_t * subscription);

// TODO: How do we allow for void * to be cast to CPP type or C type
//       depending on the user?
rmw_ret_t
rmw_take(const rmw_subscription_t * subscriber, void * ros_message);

rmw_guard_condition_t *
rmw_create_guard_condition();

rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition);

rmw_ret_t
rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition);

// TODO: May need timeout rather than or in addition to the non_blocking
rmw_ret_t
rmw_wait(rmw_subscriptions_t * subscriptions,
         rmw_guard_condition_t * guard_conditions,
         bool non_blocking);

#endif
