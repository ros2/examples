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

#include <cmath>

#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32  // i.e., Windows platform.
#include <windows.h>
#elif __APPLE__  // i.e., macOS platform.
#include <pthread.h>
#include <mach/mach_init.h>
#include <mach/mach_port.h>
#include <mach/mach_time.h>
#include <mach/thread_act.h>
#include <mach/thread_policy.h>
#include <sys/sysctl.h>
#else  // i.e., Linux platform.
  #ifdef __QNXNTO__
    #include <sys/neutrino.h>
    #include <sys/syspage.h>
  #endif
  #include <pthread.h>
#endif

#include <rclcpp/rclcpp.hpp>

namespace examples_rclcpp_cbg_executor
{

/// Retrieves the value of the given seconds parameter in std::chrono nanoseconds.
inline std::chrono::nanoseconds get_nanos_from_secs_parameter(
  const rclcpp::Node * node,
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

/// Sets the priority of the given native thread to max or min as given.
/// The exact priority value depends on the operating system. On Linux,
/// this requires elevated privileges.
/// Furthermore, if a non-negative CPU id is given, the thread is pinned
/// to that CPU.
template<typename T>
bool configure_native_thread(T native_handle, ThreadPriority priority, int cpu_id)
{
  bool success = true;
#ifdef _WIN32  // i.e., Windows platform.
  success &= (SetThreadPriority(native_handle, (priority == ThreadPriority::HIGH) ? 1 : -1) != 0);
  if (cpu_id >= 0) {
    DWORD_PTR cpuset = 1;
    cpuset <<= cpu_id;
    success &= (SetThreadAffinityMask(native_handle, cpuset) != 0);
  }
#elif __APPLE__  // i.e., macOS platform.
  thread_port_t mach_thread = pthread_mach_thread_np(native_handle);
  thread_precedence_policy_data_t precedence_policy;
  precedence_policy.importance = (priority == ThreadPriority::HIGH) ? 1 : 0;
  kern_return_t ret = thread_policy_set(
    mach_thread, THREAD_PRECEDENCE_POLICY,
    reinterpret_cast<thread_policy_t>(&precedence_policy),
    THREAD_PRECEDENCE_POLICY_COUNT);
  success &= (ret == KERN_SUCCESS);
  if (cpu_id >= 0) {
    // Pinning requires the thread to be suspended.
    ret = thread_suspend(mach_thread);
    success &= (ret == KERN_SUCCESS);
    // Wait a few milliseconds until thread is really suspended.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_affinity_policy_data_t affinity_policy;
    affinity_policy.affinity_tag = cpu_id;
    // In our experiments, the following call did not work although it
    // returned KERN_SUCCESS. If somebody knows how to fix this, please
    // open a pull request!
    ret = thread_policy_set(
      mach_thread, THREAD_AFFINITY_POLICY,
      reinterpret_cast<thread_policy_t>(&affinity_policy),
      THREAD_AFFINITY_POLICY_COUNT);
    success &= (ret == KERN_SUCCESS);
    ret = thread_resume(mach_thread);
    success &= (ret == KERN_SUCCESS);
  }
#elif __QNXNTO__  // i.e., QNX platform
  sched_param params;
  int policy;
  success &= (pthread_getschedparam(native_handle, &policy, &params) == 0);
  if (priority == ThreadPriority::HIGH) {
    // Choose a priority value slighly below the middle.
    params.sched_priority =
      (sched_get_priority_min(SCHED_FIFO) + sched_get_priority_max(SCHED_FIFO)) / 2 - 1;
  } else {
    // Choose the lowest priority in SCHED_FIFO. This might still be higher than
    // the priority of the DDS threads, which are not changed here.
    params.sched_priority = sched_get_priority_min(SCHED_FIFO);
  }
  success &= (pthread_setschedparam(native_handle, SCHED_FIFO, &params) == 0);
  if (cpu_id >= 0) {
    // Cannot set the runmask if cpu id is greater than or equal to the number of cpus
    // so will immediate return
    if (cpu_id >= _syspage_ptr->num_cpu) {
      return 0;
    }
    // run_mask is a bit mask to set which cpu a thread runs on
    // where each bit corresponds to a cpu core
    int64_t run_mask = 0x01;
    run_mask <<= cpu_id;

    // Function used to change thread affinity of thread associated with native_handle
    if (ThreadCtlExt(
        0, native_handle, _NTO_TCTL_RUNMASK,
        reinterpret_cast<void *>(run_mask)) == -1)
    {
      success &= 0;
    } else {
      success &= 1;
    }
  }
#else  // i.e., Linux platform.
  sched_param params;
  int policy;
  success &= (pthread_getschedparam(native_handle, &policy, &params) == 0);
  if (priority == ThreadPriority::HIGH) {
    // Should be a value of 49 on standard Linux platforms, which is just below
    // the default priority of 50 for threaded interrupt handling.
    params.sched_priority =
      (sched_get_priority_min(SCHED_FIFO) + sched_get_priority_max(SCHED_FIFO)) / 2 - 1;
  } else {
    // Choose the lowest priority under SCHED_FIFO. This will still be higher than
    // the priority of the DDS threads, which are not changed here. Normally
    // the DDS threads will be executed under SCHED_OTHER at nice value 0.
    // Note that changing the priority below the default user-space priority requires
    // increasing the nice level. This has not been implemented here for two reasons:
    // First, the Linux API does not allow to get the Linux-specific thread ID (TID)
    // for changing the nice value from an arbitrary pthread_t pointer, but only from the
    // current thread by gettid(). Second, a low prio Executor thread under SCHED_OTHER
    // would always get 50 ms per second due to RT throttling if not configured
    // otherwise. This would be difficult to explain in a demo.
    params.sched_priority = sched_get_priority_min(SCHED_FIFO);
  }
  success &= (pthread_setschedparam(native_handle, SCHED_FIFO, &params) == 0);
  if (cpu_id >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    success &= (pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpuset) == 0);
  }
#endif
  return success;
}

/// Sets the priority of the given thread to max or min as given. The exact
/// scheduler priority depends on the operating system. On Linux, this
/// requires elevated privileges.
/// Furthermore, if a non-negative CPU id is given, the thread is pinned
/// to that CPU.
inline bool configure_thread(std::thread & thread, ThreadPriority priority, int cpu_id)
{
  return configure_native_thread(thread.native_handle(), priority, cpu_id);
}

/// Returns the time of the given native thread handle as std::chrono
/// timestamp. This allows measuring the execution time of this thread.
template<typename T>
std::chrono::nanoseconds get_native_thread_time(T native_handle)
{
#ifdef _WIN32  // i.e., Windows platform.
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
#elif __APPLE__  // i.e., macOS platform.
  thread_port_t mach_thread = pthread_mach_thread_np(native_handle);
  thread_basic_info_data_t info;
  mach_msg_type_number_t count = THREAD_BASIC_INFO_COUNT;
  std::chrono::nanoseconds t(0);
  if (thread_info(
      mach_thread, THREAD_BASIC_INFO, reinterpret_cast<thread_info_t>(&info),
      &count) == KERN_SUCCESS)
  {
    t += std::chrono::seconds(info.user_time.seconds);
    t += std::chrono::microseconds(info.user_time.microseconds);
    t += std::chrono::seconds(info.system_time.seconds);
    t += std::chrono::microseconds(info.system_time.microseconds);
  }
  return t;
#else  // i.e., Linux platform.
  clockid_t id;
  pthread_getcpuclockid(native_handle, &id);
  timespec spec;
  clock_gettime(id, &spec);
  return std::chrono::seconds{spec.tv_sec} + std::chrono::nanoseconds{spec.tv_nsec};
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
#ifdef _WIN32  // i.e., Windows platform.
  return get_native_thread_time(GetCurrentThread());
#elif __APPLE__  // i.e., macOS platform.
  return get_native_thread_time(pthread_self());
#else  // i.e., Linux platform.
  return get_native_thread_time(pthread_self());
#endif
}

/// Calculates the average of the given vector of doubles.
inline double calc_average(const std::vector<double> & v)
{
  double avg = std::accumulate(v.begin(), v.end(), 0.0, std::plus<double>()) / v.size();
  return avg;
}

/// Calculates the standard deviation of the given vector of doubles.
inline double calc_std_deviation(const std::vector<double> & v)
{
  double mean = calc_average(v);
  double sum_squares = 0.0;
  for (const double d : v) {
    sum_squares += (d - mean) * (d - mean);
  }
  return std::sqrt(sum_squares / v.size());
}

}  // namespace examples_rclcpp_cbg_executor

#endif  // EXAMPLES_RCLCPP_CBG_EXECUTOR__UTILITIES_HPP_
