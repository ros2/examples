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

#ifndef RCLCPP__MINIMAL_COMPOSITION__VISIBILITY_H_
#define RCLCPP__MINIMAL_COMPOSITION__VISIBILITY_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define VISIBILITY_EXPORT __attribute__ ((dllexport))
    #define VISIBILITY_IMPORT __attribute__ ((dllimport))
  #else
    #define VISIBILITY_EXPORT __declspec(dllexport)
    #define VISIBILITY_IMPORT __declspec(dllimport)
  #endif

  #ifdef VISIBILITY_DLL
    #define VISIBILITY_PUBLIC VISIBILITY_EXPORT
  #else
    #define VISIBILITY_PUBLIC VISIBILITY_IMPORT
  #endif

  #define VISIBILITY_PUBLIC_TYPE VISIBILITY_PUBLIC

  #define VISIBILITY_LOCAL

#else

  #define VISIBILITY_EXPORT __attribute__ ((visibility("default")))
  #define VISIBILITY_IMPORT

  #if __GNUC__ >= 4
    #define VISIBILITY_PUBLIC __attribute__ ((visibility("default")))
    #define VISIBILITY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VISIBILITY_PUBLIC
    #define VISIBILITY_LOCAL
  #endif

  #define VISIBILITY_PUBLIC_TYPE
#endif

#if __cplusplus
}
#endif

#endif  // RCLCPP__MINIMAL_COMPOSITION__VISIBILITY_H_
