#include <cuda.h>
#include <external_dependency3.h>

extern "C"
__global__ void squared_kernel(int *in, int *out) {

  for (unsigned int i=0;i<blockDim.x;++i) {
    // /*const*/ unsigned int thread = threadIdx.x;
    out[threadIdx.x] = in[threadIdx.x] * in[threadIdx.x];
  }
};

