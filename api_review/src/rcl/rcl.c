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

#include <api_review/rmw/rmw.h>

#include <api_review/rcl/rcl.h>

rcl_ret_t
rcl_init(int argc, char **argv)
{
  rmw_ret_t ret = rmw_init();
  if (ret != RMW_RET_OK)
  {
    return ret;
  }
  return RCL_RET_OK;
}
