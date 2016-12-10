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

#ifndef RCL__VISIBILITY_HELPERS_H_
#define RCL__VISIBILITY_HELPERS_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

/* This header defines a few common visibility macros which change based only
 * on the compiler (MSVC or GCC):
 *
 *   - RCL_EXPORT: declspec statement for dllexport or visibility("default")
 *   - RCL_IMPORT: declspec statement for dllimport or empty
 *   - RCL_LOCAL: empty or visibility("hidden")
 *
 * The way this header is intended to be used is as a simple way to control
 * importing and exporting dll symbols in small, trivial packages.
 * You would first come up with a "public" macro name based on your package
 * name, e.g. if your package was called "my_package" your public macro name
 * might be "MY_PACKAGE_PUBLIC".
 * Then you would place this in each of your header files, after importing but
 * before any code:
 *
 *   #include "rcl/visibility.h"
 *
 *   #ifndef MY_PACKAGE_PUBLIC
 *   #define MY_PACKAGE_PUBLIC RCL_IMPORT
 *   #endif
 *
 * This gives you the public macro, with a default behavior of dll import.
 *
 * You can then use this public macro on all of your functions and class
 * methods like so:
 *
 *   MY_PACKAGE_PUBLIC void my_function();
 *
 * Then you would place this before all includes and code in all your source
 * files (i.e. .c or .cpp files) which include that header:
 *
 *   #include "rcl/visibility_helpers.h"
 *   #define MY_PACKAGE_PUBLIC RCL_EXPORT  // dllexport symbols in header
 *   #include "my_package/my_header.h"
 *   #undef MY_PACKAGE_PUBLIC
 *
 * This would override the default behavior for "MY_PACKAGE_PUBLIC" (which was
 * dll import) to be dll export, which is what you want when compiling the
 * library that implements this header.
 * You can exclude any macro on the definition of your functions and methods:
 *
 *   void my_function() {
 *     // ...
 *   }
 *
 * The RCL_LOCAL definition can be used on functions in your source files to
 * prevent their symbols from showing up in the resulting library, e.g.:
 *
 *   RCL_LOCAL void my_private_function() {
 *     // ...
 *   }
 */

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RCL_EXPORT __attribute__ ((dllexport))
    #define RCL_IMPORT __attribute__ ((dllimport))
  #else
    #define RCL_EXPORT __declspec(dllexport)
    #define RCL_IMPORT __declspec(dllimport)
  #endif
  #define RCL_LOCAL
#else
  #define RCL_EXPORT __attribute__ ((visibility("default")))
  #define RCL_IMPORT
  #if __GNUC__ >= 4
    #define RCL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RCL_LOCAL
  #endif
#endif

#endif  // RCL__VISIBILITY_HELPERS_H_
