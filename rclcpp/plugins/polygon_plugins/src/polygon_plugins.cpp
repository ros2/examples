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

#include <cmath>
#include <polygon_base/regular_polygon.hpp>

namespace polygon_plugins
{
class Square : public polygon_base::RegularPolygon
{
public:
  void initialize(double side_length) override
  {
    side_length_ = side_length;
  }

  double area() override
  {
    return side_length_ * side_length_;
  }

protected:
  double side_length_;
};

class Triangle : public polygon_base::RegularPolygon
{
public:
  void initialize(double side_length) override
  {
    side_length_ = side_length;
  }

  double area() override
  {
    return 0.5 * side_length_ * getHeight();
  }

  double getHeight()
  {
    return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
  }

protected:
  double side_length_;
};
}  // namespace polygon_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
