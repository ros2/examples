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

#include <cinttypes>
#include <cstdlib>
#include <ctime>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "examples_rclcpp_cbg_executor/ping_node.hpp"
#include "examples_rclcpp_cbg_executor/pong_node.hpp"
#include "examples_rclcpp_cbg_executor/utilities.hpp"

using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;
using namespace std::chrono_literals;

using examples_rclcpp_cbg_executor::PingNode;
using examples_rclcpp_cbg_executor::PongNode;
using examples_rclcpp_cbg_executor::configure_thread;
using examples_rclcpp_cbg_executor::get_thread_time;
using examples_rclcpp_cbg_executor::ThreadPriority;

// The main function composes the Ping and Pong node (depending on the arguments)
// and runs the experiment.
// See README.md for a simple architecture diagram.
// Here: rt = real-time = high scheduler priority and be = best-effort = low scheduler priority.
int main(int argc, char * argv[])
{
  const std::chrono::seconds EXPERIMENT_DURATION = 10s;

  rclcpp::init(argc, argv);

  // Create two executors within this process.
  rclcpp::executors::SingleThreadedExecutor high_prio_executor;
  rclcpp::executors::SingleThreadedExecutor low_prio_executor;

  auto pong_node = std::make_shared<PongNode>();
  high_prio_executor.add_callback_group(
    pong_node->get_high_prio_callback_group(), pong_node->get_node_base_interface());
  low_prio_executor.add_callback_group(
    pong_node->get_low_prio_callback_group(), pong_node->get_node_base_interface());

  rclcpp::Logger logger = pong_node->get_logger();

  int cpu_id = 0;

  std::mutex cv_m;
  std::condition_variable cv;

  // We instantiate two threads here, but have to suspend them right ahead in order to
  // configure the threads.
  // Platforms like OSX require the thread to the detached from any CPU before one can
  // set its priority and CPU affinity - others might require root rights.
  auto high_prio_thread = std::thread([&]() {
      std::unique_lock<std::mutex> lk(cv_m);
      cv.wait(lk);
      high_prio_executor.spin();
    });
  auto low_prio_thread = std::thread([&]() {
      std::unique_lock<std::mutex> lk(cv_m);
      cv.wait(lk);
      low_prio_executor.spin();
    });
  bool ret = configure_thread(high_prio_thread, ThreadPriority::HIGH, cpu_id);
  if (!ret) {
    RCLCPP_WARN(logger, "failed to configure high priority thread, are you root?");
  }
  ret = configure_thread(low_prio_thread, ThreadPriority::LOW, cpu_id);
  if (!ret) {
    RCLCPP_WARN(logger, "failed to configure low priority thread, are you root?");
  }
  // We've configured the threads, let's actually start the threads
  cv.notify_all();

  // Creating the threads immediately started them.
  // Therefore, get start CPU time of each thread now.
  nanoseconds high_prio_thread_begin = get_thread_time(high_prio_thread);
  nanoseconds low_prio_thread_begin = get_thread_time(low_prio_thread);

  RCLCPP_INFO(
    logger, "Running experiment from now on for %" PRId64 " s ...", EXPERIMENT_DURATION.count());

  std::this_thread::sleep_for(EXPERIMENT_DURATION);

  // Get end CPU time of each thread ...
  nanoseconds high_prio_thread_end = get_thread_time(high_prio_thread);
  nanoseconds low_prio_thread_end = get_thread_time(low_prio_thread);

  // ... and stop the experiment.
  rclcpp::shutdown();
  high_prio_thread.join();
  low_prio_thread.join();

  // Print CPU times.
  int64_t high_prio_thread_duration_ms = std::chrono::duration_cast<milliseconds>(
    high_prio_thread_end - high_prio_thread_begin).count();
  int64_t low_prio_thread_duration_ms = std::chrono::duration_cast<milliseconds>(
    low_prio_thread_end - low_prio_thread_begin).count();
  RCLCPP_INFO(
    logger, "High priority executor thread ran for %" PRId64 " ms.", high_prio_thread_duration_ms);
  RCLCPP_INFO(
    logger, "Low priority executor thread ran for %" PRId64 " ms.", low_prio_thread_duration_ms);

  return 0;
}
