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

#include <stdlib.h>
#include <memory>
#include <string>
#include <vector>
#include "class_loader/class_loader.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  std::vector<std::shared_ptr<rclcpp::Node>> nodes;  // save shared pointers !
  std::vector<std::string> lib_names;
  char * ament_prefix_path = getenv("AMENT_PREFIX_PATH");
  if (!ament_prefix_path) {
    printf("woah! AMENT_PREFIX_PATH is not set. Please source setup.bash from "
      "the install path of your ament workspace.\n");
    return 1;
  }
  const std::string lib_stem("examples_rclcpp_minimal_composition_nodes");
  std::string lib_path = std::string(ament_prefix_path) +
    std::string("/lib/") + class_loader::systemLibraryFormat(lib_stem);
  printf("opening library: %s\n", lib_path.c_str());
  auto loader = new class_loader::ClassLoader(lib_path);
  auto classes = loader->getAvailableClasses<rclcpp::Node>();
  for (auto class_name : classes) {
    printf("instantiating class %s\n", class_name.c_str());
    auto node = loader->createInstance<rclcpp::Node>(class_name);
    exec.add_node(node);
    nodes.push_back(node);
  }
  exec.spin();
  return 0;
}
