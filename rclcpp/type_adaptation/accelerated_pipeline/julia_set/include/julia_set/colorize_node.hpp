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

#ifndef JULIA_SET__COLORIZE_NODE_HPP_
#define JULIA_SET__COLORIZE_NODE_HPP_

#include <memory>

#include "cuda/julia_set.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "type_adapters/image_container.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  type_adapt_example::ImageContainer,
  sensor_msgs::msg::Image);

namespace type_adapt_example
{

/**
 * @brief
 *
 */

class ColorizeNode : public rclcpp::Node
{
public:
  explicit ColorizeNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~ColorizeNode() {}

private:
/**
* @brief Callback method on each image msg.
*
* @param img_msg Pointer to the image msg
*/
  void ColorizeCallbackCustomType(std::unique_ptr<type_adapt_example::ImageContainer> image);
  void ColorizeCallback(std::unique_ptr<sensor_msgs::msg::Image> image_msg);

  // Flag for enabling or disabling type adaptation
  const bool type_adaptation_enabled_;
  // Flag for intialization.
  bool is_initialized;
  // Juliaset prams
  JuliasetParams juliaset_params_{}; \
  // Image properties to be sent to CUDA kernel
  ImageMsgProperties img_property_{};
  // Juliaset handle
  std::unique_ptr<Juliaset> juliaset_handle_;

  // Publisher and subscriber when type_adaptation is enabled
  rclcpp::Subscription<type_adapt_example::ImageContainer>::SharedPtr custom_type_sub_ {nullptr};
  rclcpp::Publisher<type_adapt_example::ImageContainer>::SharedPtr custom_type_pub_{nullptr};

  // Publisher and subscriber when type_adaptation is disabled
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_{nullptr};
};
}  // namespace type_adapt_example
#endif  // JULIA_SET__COLORIZE_NODE_HPP_
