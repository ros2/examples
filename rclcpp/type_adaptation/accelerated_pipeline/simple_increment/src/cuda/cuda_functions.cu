#include <cstdint>

__global__
void myinc(int size, const uint8_t * source, uint8_t * destination)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index < size) {
    destination[index] = source[index] + 1;
  }
}

void cuda_compute_inc(int size, const uint8_t * source, uint8_t * destination, const cudaStream_t & stream)
{
  myinc<<<64, 64, 0, stream>>>(size, source, destination);
}

void cuda_compute_inc_inplace(int size, uint8_t * image, const cudaStream_t & stream)
{
  myinc<<<64, 64, 0, stream>>>(size, image, image);
}
