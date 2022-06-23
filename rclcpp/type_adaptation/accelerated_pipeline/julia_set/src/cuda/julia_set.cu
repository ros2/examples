/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "julia_set/cuda/julia_set.hpp"

#include <cmath>
#include <math.h>  // NOLINT - include .h without directory
#include <string>
#include <vector>
#include <stdio.h>

#include "cuda.h"  // NOLINT - include .h without directory
#include "cuda_runtime.h"  // NOLINT - include .h without directory


namespace 
{
__device__ float map_range( float input, float in_min, float in_max, float out_min, float out_max)
{
  return (((input - in_min) / (in_max - in_min)) * (out_max - out_min)) + out_min;
}

__device__ float3 hsv_to_rgb(float H, float S, float V) {
    float s = S/100;
    float v = V/100;
    float C = s*v;
    float X = C*(1-abs(fmod(H/60.0, 2)-1));
    float m = v-C;
    float r,g,b;
    if(H >= 0 && H < 60){
        r = C,g = X,b = 0;
    }
    else if(H >= 60 && H < 120){
        r = X,g = C,b = 0;
    }
    else if(H >= 120 && H < 180){
        r = 0,g = C,b = X;
    }
    else if(H >= 180 && H < 240){
        r = 0,g = X,b = C;
    }
    else if(H >= 240 && H < 300){
        r = X,g = 0,b = C;
    }
    else{
        r = C,g = 0,b = X;
    }

    float R = (r+m)*255;
    float G = (g+m)*255;
    float B = (b+m)*255;

	return (float3) {R, G, B};
}

__global__ void juliaset_kernel_composite(
    uint8_t * output, const uint8_t * input, const type_adapt_example::ImageMsgProperties img_properties, const type_adapt_example::JuliasetParams params)
{
    size_t x_idx = (blockDim.x * blockIdx.x) + threadIdx.x;
    size_t x_stride = gridDim.x * blockDim.x;

    size_t y_idx = (blockDim.y * blockIdx.y) + threadIdx.y;
    size_t y_stride = gridDim.y * blockDim.y;

    for(size_t row = y_idx; row < img_properties.height; row += y_stride) {
        for(size_t col = x_idx; col < img_properties.width; col += x_stride) {
            size_t color_idx = (row * img_properties.row_step) + (col * img_properties.color_step);

            // Map height and width on a scale of -2 to 2
            float real_part = map_range(col, params.kMinColRange, params.kMaxColRange, params.kMinXRange, params.kMaxXRange);
            float img_part = map_range(row, params.kMinRowRange, params.kMaxRowRange, params.kMinYRange, params.kMaxYRange);
            float orig_real_part = params.kStartX * cos(params.kCurrentAngle);
            float orig_img_part = params.kStartY * sin(params.kCurrentAngle);
            float new_real_part, new_img_part;
            size_t counter = 0;
            while(counter < params.kMaxIterations)
            {
                new_real_part = (real_part * real_part) - (img_part * img_part);
                new_img_part = 2 * real_part * img_part;
                
                if((real_part * real_part + img_part * img_part) > params.kBoundaryRadius * params.kBoundaryRadius)
                    break;
                real_part = new_real_part + orig_real_part;
                img_part = new_img_part + orig_img_part;
                counter++;
            }

            if(counter == params.kMaxIterations) {
                output[color_idx + img_properties.red_offset] = input[color_idx + img_properties.red_offset] / 4;
                output[color_idx + img_properties.green_offset] = input[color_idx + img_properties.green_offset] / 4;
                output[color_idx + img_properties.blue_offset] = input[color_idx + img_properties.blue_offset] / 4;
            } else {

                float h = fmod((counter * 360 / params.kMaxIterations), 360);
                float s = 100;
                float v = pow((float)counter / params.kMaxIterations,.020)*100;
                float3 color = hsv_to_rgb(h,s,v);
                output[color_idx + img_properties.red_offset] =  color.x - input[color_idx + img_properties.red_offset] / 16;
                output[color_idx + img_properties.green_offset] =  color.y - input[color_idx + img_properties.green_offset] / 16;
                output[color_idx + img_properties.blue_offset] =   color.z - input[color_idx + img_properties.blue_offset] / 16;
            }
        }
    }
}

__global__ void map_kernel(
    float * output, const type_adapt_example::ImageMsgProperties img_properties, const type_adapt_example::JuliasetParams params)
{
    size_t x_idx = (blockDim.x * blockIdx.x) + threadIdx.x;
    size_t x_stride = gridDim.x * blockDim.x;

    size_t y_idx = (blockDim.y * blockIdx.y) + threadIdx.y;
    size_t y_stride = gridDim.y * blockDim.y;

    const uint8_t kChannel = 3;

    for(size_t row = y_idx; row < img_properties.height; row += y_stride) {
        for(size_t col = x_idx; col < img_properties.width; col += x_stride) {
            size_t x_idx = (row * img_properties.width * kChannel) + (col * kChannel);
            size_t y_idx = x_idx + 1;
            size_t z_idx = y_idx + 1;

            output[x_idx] = map_range(col, params.kMinColRange, params.kMaxColRange, params.kMinXRange, params.kMaxXRange);
            output[y_idx] = map_range(row, params.kMinRowRange, params.kMaxRowRange, params.kMinYRange, params.kMaxYRange);
            output[z_idx] = 0.0;
        }
    }
}

__global__ void juliaset_kernel(size_t curr_iteration,
    float * output, const float * input, const type_adapt_example::ImageMsgProperties img_properties, const type_adapt_example::JuliasetParams params)
{
    size_t x_idx = (blockDim.x * blockIdx.x) + threadIdx.x;
    size_t x_stride = gridDim.x * blockDim.x;

    size_t y_idx = (blockDim.y * blockIdx.y) + threadIdx.y;
    size_t y_stride = gridDim.y * blockDim.y;

    const uint8_t kChannel = 3;

    for(size_t row = y_idx; row < img_properties.height; row += y_stride) {
        for(size_t col = x_idx; col < img_properties.width; col += x_stride) {
            size_t x_idx = (row * img_properties.width * kChannel) + (col * kChannel);
            size_t y_idx = x_idx + 1;
            size_t z_idx = y_idx + 1;

            float real_part = input[x_idx];
            float img_part = input[y_idx];
            float orig_real_part = params.kStartX * cos(params.kCurrentAngle);
            float orig_img_part = params.kStartY * sin(params.kCurrentAngle);
            float new_real_part, new_img_part;
          
            if(input[z_idx] == 0.0){
                if((real_part * real_part + img_part * img_part) > params.kBoundaryRadius * params.kBoundaryRadius) {
                    output[z_idx] = 1.0 + curr_iteration;
                    return;
                }
                new_real_part = (real_part * real_part) - (img_part * img_part);
                new_img_part = 2 * real_part * img_part;
                
                output[x_idx] = new_real_part + orig_real_part;
                output[y_idx] = new_img_part + orig_img_part;
                output[z_idx] = 0.0;
            } else {
                return;
            }
        }
    }
}

__global__ void colorize_kernel(
    uint8_t * output, const float * input, const type_adapt_example::ImageMsgProperties img_properties, const type_adapt_example::JuliasetParams params)
{
    size_t x_idx = (blockDim.x * blockIdx.x) + threadIdx.x;
    size_t x_stride = gridDim.x * blockDim.x;

    size_t y_idx = (blockDim.y * blockIdx.y) + threadIdx.y;
    size_t y_stride = gridDim.y * blockDim.y;

    const uint8_t kChannel = 3;

    for(size_t row = y_idx; row < img_properties.height; row += y_stride) {
        for(size_t col = x_idx; col < img_properties.width; col += x_stride) {
            
            size_t x_idx = (row * img_properties.width * kChannel) + (col * kChannel);
            size_t y_idx = x_idx + 1;
            size_t z_idx = y_idx + 1;
            
            size_t color_idx = (row * img_properties.row_step / sizeof(4)) + (col * img_properties.color_step);

            if(input[z_idx] == (float)0.0) {
                output[color_idx + img_properties.red_offset] = input[x_idx] / 4;
                output[color_idx + img_properties.green_offset] = input[y_idx] / 4;
                output[color_idx + img_properties.blue_offset] = input[z_idx] / 4;
            } else {

                float h = fmod(((input[z_idx] - 1) * 360 / params.kMaxIterations), 360);
                float s = 100;
                float v = pow((float)(input[z_idx] - 1) / params.kMaxIterations,.020)*100;
                float3 color = hsv_to_rgb(h,s,v);
                output[color_idx + img_properties.red_offset] =  color.x - input[x_idx] / 16;
                output[color_idx + img_properties.green_offset] =  color.y - input[y_idx] / 16;
                output[color_idx + img_properties.blue_offset] =   color.z - input[z_idx] / 16;
            }
        }
    }
}

}  // namespace

namespace type_adapt_example
{
Juliaset::Juliaset(ImageMsgProperties img_properties, JuliasetParams parameters): 
 image_msg_property_{img_properties},
 parameters_{parameters} {configure_kernel_execution();}

void Juliaset::configure_kernel_execution() {
    // Get the number of CUDA blocks & threads
    size_t num_blocks_x = (image_msg_property_.width + num_threads_per_block_x_ - 1) / 
                        num_threads_per_block_x_;
    size_t num_blocks_y = (image_msg_property_.height + num_threads_per_block_y_ - 1) /
                        num_threads_per_block_y_;

    num_of_blocks_ = dim3(num_blocks_x, num_blocks_y, 1);
    threads_per_block_ = dim3(num_threads_per_block_x_, num_threads_per_block_y_, 1);
}

void Juliaset::compute_juliaset_composite(float & current_angle, u_int8_t * image, const cudaStream_t & stream)
{
    parameters_.kCurrentAngle = current_angle;
    // Invoke CUDA kernel
    juliaset_kernel_composite<<<num_of_blocks_, threads_per_block_, 0, stream>>>(image,
                                                                            image,
                                                                            image_msg_property_,
                                                                            parameters_); 
}

void Juliaset::map(float * out_mat, const cudaStream_t & stream)
{
    // Invoke CUDA kernel
    map_kernel<<<num_of_blocks_, threads_per_block_, 0, stream>>>(out_mat,
                                                                image_msg_property_,
                                                                parameters_);
}

void Juliaset::compute_juliaset_pipeline(
    size_t curr_iteration, float & current_angle, float * image, const cudaStream_t & stream)
{
    parameters_.kCurrentAngle = current_angle;
    // Invoke CUDA kernel
    juliaset_kernel<<<num_of_blocks_, threads_per_block_, 0, stream>>>(curr_iteration,
                                                                    image,
                                                                    image,
                                                                    image_msg_property_,
                                                                    parameters_);
  
}

void Juliaset::colorize(
    uint8_t * output, const float * input, const cudaStream_t & stream)
{
    // Invoke CUDA kernel
    colorize_kernel<<<num_of_blocks_, threads_per_block_, 0, stream>>>(output,
                                                                    input,
                                                                    image_msg_property_,
                                                                    parameters_);
}

}  // type_adapt_example
