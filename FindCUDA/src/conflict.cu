
#include <external_dependency.h>

__global__ void cubed_kernel(int *in, int *out) {

  for (unsigned int i=0;i<blockDim.x;++i) {
    // /*const*/ unsigned int thread = threadIdx.x;
    out[threadIdx.x] = in[threadIdx.x] * in[threadIdx.x] * in[threadIdx.x];
  }
};

extern "C"
void cubed(int* in, int* out, int dim) {
  // Setup kernel problem size
  dim3 blocksize(dim,1,1);
  dim3 gridsize(1,1,1);

  // Call kernel
  cubed_kernel<<<gridsize, blocksize>>>(in, out);
}

