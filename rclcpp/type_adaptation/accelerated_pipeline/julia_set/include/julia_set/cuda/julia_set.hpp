/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef TYPE_ADAPT_EXAMPLE__JULIA_SET_HPP_
#define TYPE_ADAPT_EXAMPLE__JULIA_SET_HPP_

#include <cstdint>

#include "cuda.h"  // NOLINT - include .h without directory
#include "cuda_runtime.h"  // NOLINT - include .h without directory

#include <string>

namespace type_adapt_example
{

/**
* @brief Struct that holds relevant Image properties
*/
struct ImageMsgProperties
{
  unsigned int row_step{0};  // Length of a row
  unsigned int height{0};  // Height of the RGB image
  unsigned int width{0};  // Width of the RGB image
  std::string encoding{""};  // Data format of the RGB image
  unsigned int red_offset{0};  // Position of red point for one pixel
  unsigned int green_offset{0};  // Position of green point for one pixel
  unsigned int blue_offset{0};  // Position of blue point for one pixel
  unsigned int color_step{0};  // Number of points per pixel
};

struct JuliasetParams
{
  float kCurrentAngle{0.0};
  float kMinXRange{-2.5};
  float kMaxXRange{2.5};
  float kMinYRange{-2.5};
  float kMaxYRange{2.5};
  float kStartX{0.7885};
  float kStartY{0.7885};
  float kBoundaryRadius{16.0};
  size_t kMaxIterations{50};
  uint32_t kMinColRange{0};
  uint32_t kMaxColRange{0};
  uint32_t kMinRowRange{0};
  uint32_t kMaxRowRange{0};
};

class Juliaset
{
public:
  explicit Juliaset(ImageMsgProperties img_properties, JuliasetParams parameters);
  ~Juliaset() = default;

  void compute_juliaset_composite(
    float & current_angle, u_int8_t * image, const cudaStream_t & stream);

  void map(float * out_mat, const cudaStream_t & stream);

  void compute_juliaset_pipeline(
    size_t curr_iteration, float & current_angle, float * image, const cudaStream_t & stream);

  void colorize(
    uint8_t * output, const float * input, const cudaStream_t & stream);

private:
  void configure_kernel_execution();
  // Properties of image msg from ROS
  ImageMsgProperties image_msg_property_{};
  // Params for Juliaset calculations
  JuliasetParams parameters_{};
  // The number of CUDA threads per block in the x direction
  const int num_threads_per_block_x_{32};
  // The number of CUDA threads per block in the y direction
  const int num_threads_per_block_y_{32};
  // The number of blocks for kernel execution
  dim3 num_of_blocks_;
  // The number of CUDA threads per block
  dim3 threads_per_block_;
};

}
#endif  // TYPE_ADAPT_EXAMPLE__JULIA_SET_HPP_
