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

#include <nvToolsExt.h>  // NOLINT

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "type_adapters/image_container.hpp"
#include "simple_increment/cuda/cuda_functions.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  type_adaptation::example_type_adapters::ImageContainer,
  sensor_msgs::msg::Image);

namespace type_adaptation
{
namespace simple_increment
{

class IncNode : public rclcpp::Node
{
public:
  explicit IncNode(rclcpp::NodeOptions options)
  : rclcpp::Node("inc_node", options.use_intra_process_comms(true)),
    proc_count_(declare_parameter<int>("proc_count", 1)),
    inplace_enabled_(declare_parameter<bool>("inplace_enabled", false)),
    type_adaptation_enabled_(declare_parameter<bool>("type_adaptation_enabled", true))
  {
    RCLCPP_INFO(
      get_logger(), "Setting up node to run with inplace_enabled %s with proc count %d",
      inplace_enabled_ ? "YES" : "NO", proc_count_);
    RCLCPP_INFO(
      get_logger(), "Type adaptation enabled: %s", type_adaptation_enabled_ ? "YES" : "NO");

    if (type_adaptation_enabled_) {
      custom_type_sub_ =
        create_subscription<type_adaptation::example_type_adapters::ImageContainer>(
        "image_in", 1, std::bind(&IncNode::custom_type_callback, this, std::placeholders::_1));
      custom_type_pub_ = create_publisher<type_adaptation::example_type_adapters::ImageContainer>(
        "image_out", 1);
    } else {
      sub_ =
        create_subscription<sensor_msgs::msg::Image>(
        "image_in", 1, std::bind(&IncNode::callback, this, std::placeholders::_1));
      pub_ = create_publisher<sensor_msgs::msg::Image>("image_out", 1);
    }
  }

  void custom_type_callback(
    std::unique_ptr<type_adaptation::example_type_adapters::ImageContainer> image)
  {
    nvtxRangePushA("IncNode: Image custom_type_callback");
    for (int i = 0; i < proc_count_; i++) {
      if (inplace_enabled_) {
        cuda_compute_inc_inplace(
          image->size_in_bytes(), image->cuda_mem(),
          image->cuda_stream()->stream());
      } else {
        auto copy = std::make_unique<type_adaptation::example_type_adapters::ImageContainer>(
          image->header(), image->height(),
          image->width(), image->encoding(), image->step(), image->cuda_stream());
        cuda_compute_inc(
          image->size_in_bytes(), image->cuda_mem(),
          copy->cuda_mem(), copy->cuda_stream()->stream());
        image = std::move(copy);
      }
    }
    custom_type_pub_->publish(std::move(image));
    nvtxRangePop();
  }

  void callback(std::unique_ptr<sensor_msgs::msg::Image> image_msg)
  {
    nvtxRangePushA("IncNode: Image callback");
    using ImageContainer = type_adaptation::example_type_adapters::ImageContainer;
    std::unique_ptr<ImageContainer> image = std::make_unique<ImageContainer>(std::move(image_msg));
    for (int i = 0; i < proc_count_; i++) {
      if (inplace_enabled_) {
        cuda_compute_inc_inplace(
          image->size_in_bytes(), image->cuda_mem(),
          image->cuda_stream()->stream());
      } else {
        auto copy = std::make_unique<ImageContainer>(
          image->header(), image->height(), image->width(), image->encoding(),
          image->step(), image->cuda_stream());
        cuda_compute_inc(
          image->size_in_bytes(), image->cuda_mem(),
          copy->cuda_mem(), copy->cuda_stream()->stream());
        image = std::move(copy);
      }
    }
    // Convert in-place before publishing to "disable" type adaptation
    sensor_msgs::msg::Image image_msg_out;
    image->get_sensor_msgs_image(image_msg_out);
    pub_->publish(image_msg_out);
    nvtxRangePop();
  }

private:
  // Publisher and subscriber when type_adaptation is enabled
  rclcpp::Subscription<type_adaptation::example_type_adapters::ImageContainer>::SharedPtr
    custom_type_sub_ {nullptr};
  rclcpp::Publisher<type_adaptation::example_type_adapters::ImageContainer>::SharedPtr
    custom_type_pub_{nullptr};

  // Publisher and subscriber when type_adaptation is disabled
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_{nullptr};

  const int proc_count_;
  const bool inplace_enabled_;
  const bool type_adaptation_enabled_;
};

}  // namespace simple_increment
}  // namespace type_adaptation

RCLCPP_COMPONENTS_REGISTER_NODE(type_adaptation::simple_increment::IncNode)
