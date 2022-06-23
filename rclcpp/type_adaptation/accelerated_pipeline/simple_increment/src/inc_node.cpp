// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "type_adapt_example/nvidia_cuda_sensor_msgs_image_type_adapter.hpp"
#include "type_adapt_example/cuda_functions.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  type_adapt_example::ROSNvidiaCudaContainer,
  sensor_msgs::msg::Image);

namespace type_adapt_example
{

class IncNode : public rclcpp::Node
{
public:
  explicit IncNode(rclcpp::NodeOptions options)
  : rclcpp::Node("inc_node", options.use_intra_process_comms(true)),
    proc_count_(declare_parameter<int>("proc_count", 1)),
    inplace_enabled_(declare_parameter<bool>("inplace_enabled", false))
  {
    RCLCPP_INFO(
      get_logger(), "Setting up node to run with inplace_enabled %s with proc count %d",
      inplace_enabled_ ? "YES" : "NO",
      proc_count_);
    auto callback =
      [this](std::unique_ptr<type_adapt_example::ROSNvidiaCudaContainer> image) {
        for (int i = 0; i < proc_count_; i++) {
          if (inplace_enabled_) {
            cuda_compute_inc_inplace(
              image->size_in_bytes(), image->cuda_mem(),
              image->cuda_stream()->stream());
          } else {
            auto copy = std::make_unique<type_adapt_example::ROSNvidiaCudaContainer>(
              image->header(), image->height(), image->width(), image->encoding(),
              image->step(), image->cuda_stream());
            cuda_compute_inc(
              image->size_in_bytes(), image->cuda_mem(),
              copy->cuda_mem(), copy->cuda_stream()->stream());
            image = std::move(copy);
          }
        }

        pub_->publish(std::move(image));
      };

    // This is the input into the pipeline from an external source
    sub_ =
      create_subscription<type_adapt_example::ROSNvidiaCudaContainer>("image_in", 1, callback);

    // This is the publication to the rest of the GPU pipeline
    pub_ = create_publisher<type_adapt_example::ROSNvidiaCudaContainer>("image_out", 1);
  }

private:
  rclcpp::Subscription<type_adapt_example::ROSNvidiaCudaContainer>::SharedPtr sub_;
  rclcpp::Publisher<type_adapt_example::ROSNvidiaCudaContainer>::SharedPtr pub_;

  const int proc_count_;
  const bool inplace_enabled_;
};

}  // namespace type_adapt_example

RCLCPP_COMPONENTS_REGISTER_NODE(type_adapt_example::IncNode)
