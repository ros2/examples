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

/// The main function composes a Ping node and a Pong node in one OS process
/// and runs the experiment. See README.md for an architecture diagram.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create two executors within this process.
  rclcpp::executors::SingleThreadedExecutor high_prio_executor;
  rclcpp::executors::SingleThreadedExecutor low_prio_executor;

  // Create Ping node instance and add it to high-prio executor.
  auto ping_node = std::make_shared<PingNode>();
  high_prio_executor.add_node(ping_node);

  // Create Pong node instance and add it the one of its callback groups
  // to the high-prio executor and the other to the low-prio executor.
  auto pong_node = std::make_shared<PongNode>();
  high_prio_executor.add_callback_group(
    pong_node->get_high_prio_callback_group(), pong_node->get_node_base_interface());
  low_prio_executor.add_callback_group(
    pong_node->get_low_prio_callback_group(), pong_node->get_node_base_interface());

  rclcpp::Logger logger = pong_node->get_logger();

  // Create a thread for each of the two executors ...
  auto high_prio_thread = std::thread(
    [&]() {
      high_prio_executor.spin();
    });
  auto low_prio_thread = std::thread(
    [&]() {
      low_prio_executor.spin();
    });

  // ... and configure them accordinly as high and low prio and pin them to the
  // first CPU. Hence, the two executors compete about this computational resource.
  const int CPU_ZERO = 0;
  bool ret = configure_thread(high_prio_thread, ThreadPriority::HIGH, CPU_ZERO);
  if (!ret) {
    RCLCPP_WARN(logger, "Failed to configure high priority thread, are you root?");
  }
  ret = configure_thread(low_prio_thread, ThreadPriority::LOW, CPU_ZERO);
  if (!ret) {
    RCLCPP_WARN(logger, "Failed to configure low priority thread, are you root?");
  }

  // Creating the threads immediately started them.
  // Therefore, get start CPU time of each thread now.
  nanoseconds high_prio_thread_begin = get_thread_time(high_prio_thread);
  nanoseconds low_prio_thread_begin = get_thread_time(low_prio_thread);

  const std::chrono::seconds EXPERIMENT_DURATION = 10s;
  RCLCPP_INFO_STREAM(
    logger, "Running experiment from now on for " << EXPERIMENT_DURATION.count() << " seconds ...");
  std::this_thread::sleep_for(EXPERIMENT_DURATION);

  // Get end CPU time of each thread ...
  nanoseconds high_prio_thread_end = get_thread_time(high_prio_thread);
  nanoseconds low_prio_thread_end = get_thread_time(low_prio_thread);

  // ... and stop the experiment.
  rclcpp::shutdown();
  high_prio_thread.join();
  low_prio_thread.join();

  ping_node->print_statistics(EXPERIMENT_DURATION);

  // Print CPU times.
  int64_t high_prio_thread_duration_ms = std::chrono::duration_cast<milliseconds>(
    high_prio_thread_end - high_prio_thread_begin).count();
  int64_t low_prio_thread_duration_ms = std::chrono::duration_cast<milliseconds>(
    low_prio_thread_end - low_prio_thread_begin).count();
  RCLCPP_INFO(
    logger, "High priority executor thread ran for %" PRId64 "ms.", high_prio_thread_duration_ms);
  RCLCPP_INFO(
    logger, "Low priority executor thread ran for %" PRId64 "ms.", low_prio_thread_duration_ms);

  return 0;
}
