#include "external_dependency.h"
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>

extern "C"
void cubed(int* in, int* out, int dim);

extern "C"
int doit_cubed()
{
  cudaFree(0);
  CHECK_CUDA_ERROR();

  int h_val[DIM];
  int h_result[DIM];

  for(int i = 0; i < DIM; ++i)
    h_val[i] = i;

  // Allocate device memory
  unsigned int size = sizeof(int) * DIM;
  int* d_val;
  cudaMalloc((void**)&d_val, size);
  CHECK_CUDA_ERROR();

  int* d_result;
  cudaMalloc((void**)&d_result, size);
  CHECK_CUDA_ERROR();

  // Send input to device
  cudaMemcpy(d_val, h_val, size, cudaMemcpyHostToDevice);
  CHECK_CUDA_ERROR();

  // Call the kernel wrapper
  cubed(d_val, d_result, DIM);
  CHECK_CUDA_ERROR();

  // Get back results
  cudaMemcpy(h_result, d_result, size, cudaMemcpyDeviceToHost);
  CHECK_CUDA_ERROR();

  for(int i = 0; i < DIM; ++i)
    printf("%d ^ 3 = %d\n", h_val[i], h_result[i]);

  // Free memory
  cudaFree((void*)d_val);
  CHECK_CUDA_ERROR();

  cudaFree((void*)d_result);
  CHECK_CUDA_ERROR();

  return 0;
}


