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

#ifndef RCL_RCL_TYPES_H_
#define RCL_RCL_TYPES_H_

#include <rmw/types.h>

typedef rmw_ret_t rcl_ret_t;
#define RCL_RET_OK RMW_RET_OK

typedef rmw_node_t rcl_node_t;
typedef rmw_publisher_t rcl_publisher_t;
typedef rmw_subscription_t rcl_subscription_t;
typedef rmw_guard_condition_t rcl_guard_condition_t;
typedef rmw_subscriptions_t rcl_subscriptions_t;
typedef rmw_guard_conditions_t rcl_guard_conditions_t;

typedef struct rcl_callback_group_t
{
  rcl_node_t * * node;
} rcl_callback_group_t;

typedef struct rcl_subscription_info_t
{
  rcl_subscription_t * * subscription;
  rcl_callback_group_t * * callback_group;
} rcl_subscription_info_t;

typedef struct rcl_timer_info_t
{
  rcl_guard_condition_t * * guard_condition;
  rcl_callback_group_t * * callback_group;
} rcl_timer_info_t;

typedef struct rcl_executor_helper_t
{
  // TODO: fill with something
} rcl_executor_helper_t;

typedef struct rcl_any_executable_t
{
  rcl_subscription_info_t * * subscription_info;
  rcl_timer_info_t * * timer_info;
} rcl_any_executable_t;

#endif  /* RCL_RCL_TYPES_H_ */
