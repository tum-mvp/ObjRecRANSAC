
#include <stdio.h>
#include <cuda.h>
#include <fstream>
#include "cuda-driver-error.h"
#include "external_dependency3.h"

CUcontext g_cuContext = 0;
CUfunction g_cuda_squared_kernel = 0;
const char* g_ptx = 0;

CUresult initCUDA()
{
  CUdevice cuDevice;
  CUmodule cuModule;
  int major, minor, devID = 0;
  char deviceName[256];

	// link to cuda driver dynamically 
  cutilDrvCheckCallNoSync(cuInit( 0 ));

  int deviceCount = 0;
  cutilDrvCheckCallNoSync(cuDeviceGetCount(&deviceCount));
  if (deviceCount == 0) {
    fprintf(stderr, "No devices supporting CUDA detected, exiting...\n");
  } else {
    printf("> Found %d devices\n", deviceCount);
  }
  
  // pick up device with zero ordinal
  cutilDrvCheckCallNoSync(cuDeviceGet(&cuDevice, devID));

	// get compute capabilities and the devicename
	cutilDrvCheckCallNoSync( cuDeviceComputeCapability(&major, &minor, cuDevice) );
	cutilDrvCheckCallNoSync( cuDeviceGetName(deviceName, 256, cuDevice) );
	printf("> Device %d: \"%s\" with Compute %d.%d capability\n", cuDevice, deviceName, major, minor);

  // Create the cuda context
  cutilDrvCheckCallNoSync(cuCtxCreate(&g_cuContext, 0, cuDevice));

  // setup JIT compilation options and perform compilation
  {
    // in this branch we use compilation with parameters
    const unsigned int jitNumOptions = 3;
    CUjit_option *jitOptions = new CUjit_option[jitNumOptions];
    void **jitOptVals = new void*[jitNumOptions];

    // set up size of compilation log buffer
    jitOptions[0] = CU_JIT_INFO_LOG_BUFFER_SIZE_BYTES;
    int jitLogBufferSize = 1024;
    jitOptVals[0] = (void *)(size_t)jitLogBufferSize;

    // set up pointer to the compilation log buffer
    jitOptions[1] = CU_JIT_INFO_LOG_BUFFER;
    char *jitLogBuffer = new char[jitLogBufferSize];
    jitOptVals[1] = jitLogBuffer;

		// set up pointer to set the Maximum # of registers for a particular kernel
		jitOptions[2] = CU_JIT_MAX_REGISTERS;
		int jitRegCount = 32;
		jitOptVals[2] = (void *)(size_t)jitRegCount;

    // compile with set parameters
    printf("> Compiling CUDA module\n");

    cutilDrvCheckCallNoSync(cuModuleLoadDataEx(&cuModule, g_ptx, jitNumOptions, jitOptions, (void **)jitOptVals));

    printf("> PTX JIT log:\n%s\n", jitLogBuffer);
        
    delete [] jitOptions;
    delete [] jitOptVals;
    delete [] jitLogBuffer;
  }

  // retrieve CUDA function from the compiled module
  cutilDrvCheckCallNoSync(cuModuleGetFunction(&g_cuda_squared_kernel, cuModule, "squared_kernel"));

  return CUDA_SUCCESS;
}

int main( int argc, char **argv )
{
  // Load the PTX
  std::ifstream file("cuda_compile_ptx_generated_test_ptx.cu.ptx");
  if (!file) {
    fprintf(stderr, "Error opening ptx file\n");
    return 2;
  }
  std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  g_ptx = str.c_str();

  cutilDrvCheckCallNoSync(initCUDA());

  int h_val[DIM];
  int h_result[DIM];

  for(int i = 0; i < DIM; ++i)
    h_val[i] = i;

  // Allocate device memory
  unsigned int size = sizeof(int) * DIM;
  CUdeviceptr d_val;
  cutilDrvCheckCallNoSync(cuMemAlloc(&d_val, size));

  CUdeviceptr d_result;
  cutilDrvCheckCallNoSync(cuMemAlloc(&d_result, size));

  // Send input to device
  cutilDrvCheckCallNoSync(cuMemcpyHtoD(d_val, h_val, size));

  // Call the kernel wrapper
  void *args[2] = { &d_val, &d_result};
  // Blocksize is (DIM, 1, 1)
  // Gridsize is (1,1,1)
  cutilDrvCheckCallSync(cuLaunchKernel(g_cuda_squared_kernel,
                                      1,1,1,     // Gridsize
                                      DIM, 1, 1, // Blocksize
                                      0,         // sharedMemBytes
                                      0,         // CUstream hStream
                                      args,      // kernelParams
                                      0 ));       // extra

  // Get back results
  cuMemcpyDtoH(h_result, d_result, size);

  for(int i = 0; i < DIM; ++i)
    printf("%d ^ 2 = %d\n", h_val[i], h_result[i]);

  // Free memory
  cutilDrvCheckCallNoSync(cuMemFree(d_val));
  cutilDrvCheckCallNoSync(cuMemFree(d_result));
  cutilDrvCheckCallNoSync(cuCtxDetach(g_cuContext));

  return 0;
};

