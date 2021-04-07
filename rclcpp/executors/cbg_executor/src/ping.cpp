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
#include "examples_rclcpp_cbg_executor/utilities.hpp"

using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;
using namespace std::chrono_literals;

using examples_rclcpp_cbg_executor::PingNode;
using examples_rclcpp_cbg_executor::configure_thread;
using examples_rclcpp_cbg_executor::get_thread_time;
using examples_rclcpp_cbg_executor::ThreadPriority;

/// The main function puts a Ping node in one OS process and runs the
/// experiment. See README.md for an architecture diagram.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create one executor within this process.
  rclcpp::executors::SingleThreadedExecutor high_prio_executor;

  // Create Ping node instance and add it to high-prio executor.
  auto ping_node = std::make_shared<PingNode>();
  high_prio_executor.add_node(ping_node);

  rclcpp::Logger logger = ping_node->get_logger();

  // Create a thread for the executor ...
  auto high_prio_thread = std::thread(
    [&]() {
      high_prio_executor.spin();
    });

  // ... and configure it accordinly as high prio and pin it to the first CPU.
  const int CPU_ZERO = 0;
  bool ret = configure_thread(high_prio_thread, ThreadPriority::HIGH, CPU_ZERO);
  if (!ret) {
    RCLCPP_WARN(logger, "Failed to configure high priority thread, are you root?");
  }

  const std::chrono::seconds EXPERIMENT_DURATION = 10s;
  RCLCPP_INFO_STREAM(
    logger, "Running experiment from now on for " << EXPERIMENT_DURATION.count() << " seconds ...");
  std::this_thread::sleep_for(EXPERIMENT_DURATION);

  // ... and stop the experiment.
  rclcpp::shutdown();
  high_prio_thread.join();

  ping_node->print_statistics(EXPERIMENT_DURATION);

  return 0;
}
