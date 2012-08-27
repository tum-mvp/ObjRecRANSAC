#ifndef EXTERNDEPENDENCY__H
#define EXTERNDEPENDENCY__H

#include "external_dependency3.h"
typedef unsigned int Size; 

#define CHECK_CUDA_ERROR() \
  { \
    cudaThreadSynchronize(); \
    cudaError_t error = cudaGetLastError(); \
    if(error != cudaSuccess) { \
      printf("error (%s: line %d): %s\n", __FILE__, __LINE__, cudaGetErrorString(error)); \
      return 1; \
    } \
  }
#endif
