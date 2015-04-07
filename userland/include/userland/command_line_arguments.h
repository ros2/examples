// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef __userland__command_line_arguments__h__
#define __userland__command_line_arguments__h__

#include <iostream>

#include <string>


const char * valid_message_args[] = {"counter", "all_primitive", "all_static_array", "all_dynamic_array", "nested", "string", "all_primitive_defaults", "all_builtin"};

void print_message_usage()
{
  std::cout << "  --msg MSG_TYPE" << std::endl;
  std::cout << "\t\tValid MSG_TYPEs are:";
  for (const char * msg : valid_message_args) {
    std::cout << " " << msg;
  }
  std::cout << std::endl;
}

bool has_argument(char ** begin, char ** end, const std::string& name)
{
  while (begin != end) {
    if (name == *begin) return true;
    ++begin;
  }
  return false;
}

const char * get_named_argument(char ** begin, char ** end, const std::string& name, const char * default_value = nullptr)
{
  while (begin != end) {
    if (name == *begin && (begin + 1) != end) return *(begin + 1);
    ++begin;
  }
  return default_value;
}

#endif  // __userland__command_line_arguments__h__
