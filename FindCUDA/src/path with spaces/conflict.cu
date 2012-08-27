
#include <external_dependency.h>

__global__ void squared_kernel(int *in, int *out) {

  for (unsigned int i=0;i<blockDim.x;++i) {
    // /*const*/ unsigned int thread = threadIdx.x;
    out[threadIdx.x] = in[threadIdx.x] * in[threadIdx.x];
  }
};

extern "C"
void squared(int* in, int* out, int dim) {
  // Setup kernel problem size
  dim3 blocksize(dim,1,1);
  dim3 gridsize(1,1,1);

  // Call kernel
  squared_kernel<<<gridsize, blocksize>>>(in, out);
}

