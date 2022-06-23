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

#ifndef TYPE_ADAPT_EXAMPLE__CUDA_FUNCTIONS_HPP_
#define TYPE_ADAPT_EXAMPLE__CUDA_FUNCTIONS_HPP_

#include "cuda.h"  // NOLINT

void cuda_compute_xor(int n, uint8_t * image, const cudaStream_t & stream);

void cuda_compute_inc(
  int size, const uint8_t * source, uint8_t * destination,
  const cudaStream_t & stream);

void cuda_compute_inc_inplace(int size, uint8_t * image, const cudaStream_t & stream);

#endif  // TYPE_ADAPT_EXAMPLE__CUDA_FUNCTIONS_HPP_
