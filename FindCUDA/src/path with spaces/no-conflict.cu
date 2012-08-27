
#include <external_dependency.h>

__global__ void times2_kernel(int *in, int *out) {

  for (unsigned int i=0;i<blockDim.x;++i) {
    // /*const*/ unsigned int thread = threadIdx.x;
    out[threadIdx.x] = in[threadIdx.x] * 2;
  }
};

extern "C"
void times2(int* in, int* out, int dim) {
  // Setup kernel problem size
  dim3 blocksize(dim,1,1);
  dim3 gridsize(1,1,1);

  // Call kernel
  times2_kernel<<<gridsize, blocksize>>>(in, out);
}

