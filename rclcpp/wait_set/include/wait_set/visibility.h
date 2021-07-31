// Copyright 2021, Apex.AI Inc.
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

#ifndef WAIT_SET__VISIBILITY_H_
#define WAIT_SET__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define WAIT_SET_EXPORT __attribute__ ((dllexport))
    #define WAIT_SET_IMPORT __attribute__ ((dllimport))
  #else
    #define WAIT_SET_EXPORT __declspec(dllexport)
    #define WAIT_SET_IMPORT __declspec(dllimport)
  #endif

  #ifdef WAIT_SET_DLL
    #define WAIT_SET_PUBLIC WAIT_SET_EXPORT
  #else
    #define WAIT_SET_PUBLIC WAIT_SET_IMPORT
  #endif

  #define WAIT_SET_PUBLIC_TYPE WAIT_SET_PUBLIC

  #define WAIT_SET_LOCAL

#else

  #define WAIT_SET_EXPORT __attribute__ ((visibility("default")))
  #define WAIT_SET_IMPORT

  #if __GNUC__ >= 4
    #define WAIT_SET_PUBLIC __attribute__ ((visibility("default")))
    #define WAIT_SET_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WAIT_SET_PUBLIC
    #define WAIT_SET_LOCAL
  #endif

  #define WAIT_SET_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // WAIT_SET__VISIBILITY_H_
