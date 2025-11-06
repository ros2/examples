// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include <pluginlib/class_loader.hpp>
#include <polygon_base/regular_polygon.hpp>

int main(int argc, char ** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base",
    "polygon_base::RegularPolygon");

  try {
    std::shared_ptr<polygon_base::RegularPolygon> triangle =
      poly_loader.createSharedInstance("awesome_triangle");
    triangle->initialize(10.0);

    std::shared_ptr<polygon_base::RegularPolygon> square =
      poly_loader.createSharedInstance("polygon_plugins::Square");
    square->initialize(10.0);

    std::cout << "Triangle area: " << triangle->area() << std::endl;
    std::cout << "Square area: " << square->area() << std::endl
  } catch(pluginlib::PluginlibException & ex) {
    std::cout << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
  }

  return 0;
}
