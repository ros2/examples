// Copyright (c) 2020 Robert Bosch GmbH
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

#ifndef EXAMPLES_RCLCPP_CBG_EXECUTOR__UTILITIES_HPP_
#define EXAMPLES_RCLCPP_CBG_EXECUTOR__UTILITIES_HPP_

#include <chrono>
#include <string>
#include <thread>

#ifndef _WIN32  // i.e., POSIX platform.
#include <pthread.h>
#else  // i.e., Windows platform.
#include <windows.h>
#endif

#include <rclcpp/rclcpp.hpp>

namespace examples_rclcpp_cbg_executor
{

/// Retrieves the value of the given seconds parameter in std::chrono nanoseconds.
inline std::chrono::nanoseconds get_nanos_from_secs_parameter(
  rclcpp::Node * node,
  const std::string & name)
{
  double seconds = 0.0;
  node->get_parameter(name, seconds);
  auto nanos = std::chrono::nanoseconds(static_cast<int64_t>(seconds * 1000000000.0));
  return nanos;
}

/// Enum for simple configuration of threads in two priority classes.
enum class ThreadPriority
{
  LOW,
  HIGH
};

/// Sets the priority of the given thread to max or min priority (in the SCHED_FIFO real-time
/// policy) and pins the thread to the given cpu (if cpu_id >= 0).
template<typename T>
bool configure_native_thread(T native_handle, ThreadPriority priority, int cpu_id)
{
  bool success = true;
#ifndef _WIN32  // i.e., POSIX platform.
  sched_param params;
  int policy;
  success &= (pthread_getschedparam(native_handle, &policy, &params) == 0);
  if (priority == ThreadPriority::HIGH) {
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
  } else {
    params.sched_priority = sched_get_priority_min(SCHED_FIFO);
  }

  success &= (pthread_setschedparam(native_handle, SCHED_FIFO, &params) == 0);

  if (cpu_id >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    success &= (pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpuset) == 0);
  }
#else  // i.e., Windows platform.
  success &= (SetThreadPriority(native_handle, (priority == ThreadPriority::HIGH) ? 1 : -1) != 0);
  if (cpu_id >= 0) {
    DWORD_PTR cpuset = 1;
    cpuset <<= cpu_id;
    success &= (SetThreadAffinityMask(native_handle, cpuset) != 0);
  }
#endif
  return success;
}

/// Sets the priority of the given thread to max or min priority (in the SCHED_FIFO real-time
/// policy) and pins the thread to the given cpu (if cpu_id >= 0).
inline bool configure_thread(std::thread & thread, ThreadPriority priority, int cpu_id)
{
  return configure_native_thread(thread.native_handle(), priority, cpu_id);
}

/// Returns the time of the given native thread handle as std::chrono
/// timestamp. This allows measuring the execution time of this thread.
template<typename T>
std::chrono::nanoseconds get_native_thread_time(T native_handle)
{
#ifndef _WIN32  // i.e., POSIX platform.
  clockid_t id;
  pthread_getcpuclockid(native_handle, &id);
  timespec spec;
  clock_gettime(id, &spec);
  return std::chrono::seconds{spec.tv_sec} + std::chrono::nanoseconds{spec.tv_nsec};
#else  // i.e., Windows platform.
  FILETIME creation_filetime;
  FILETIME exit_filetime;
  FILETIME kernel_filetime;
  FILETIME user_filetime;
  GetThreadTimes(
    native_handle, &creation_filetime, &exit_filetime, &kernel_filetime, &user_filetime);
  ULARGE_INTEGER kernel_time;
  kernel_time.LowPart = kernel_filetime.dwLowDateTime;
  kernel_time.HighPart = kernel_filetime.dwHighDateTime;
  ULARGE_INTEGER user_time;
  user_time.LowPart = user_filetime.dwLowDateTime;
  user_time.HighPart = user_filetime.dwHighDateTime;
  std::chrono::nanoseconds t(100);  // Unit in FILETIME is 100ns.
  t *= (kernel_time.QuadPart + user_time.QuadPart);
  return t;
#endif
}

/// Returns the time of the given thread as std::chrono timestamp.
/// This allows measuring the execution time of this thread.
inline std::chrono::nanoseconds get_thread_time(std::thread & thread)
{
  return get_native_thread_time(thread.native_handle());
}

/// Returns the time of the current thread as std::chrono timestamp.
/// This allows measuring the execution time of this thread.
inline std::chrono::nanoseconds get_current_thread_time()
{
#ifndef _WIN32  // i.e., POSIX platform.
  return get_native_thread_time(pthread_self());
#else  // i.e., Windows platform.
  return get_native_thread_time(GetCurrentThread());
#endif
}

}  // namespace examples_rclcpp_cbg_executor

#endif  // EXAMPLES_RCLCPP_CBG_EXECUTOR__UTILITIES_HPP_
