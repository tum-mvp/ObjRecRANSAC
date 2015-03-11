#include "AcceptHypothesisAlgo.h"
#include <cuda.h>
#include <cuda_runtime.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>

// upper bound for the registers used by this kernel. This is printed while compiling the cu file.
static const int kernel_reg_count = 64;

inline int _ConvertSMVer2Cores(int major, int minor)
{
    // Defines for GPU Architecture types (using the SM version to determine the # of cores per SM
    typedef struct
    {
        int SM; // 0xMm (hexidecimal notation), M = SM Major version, and m = SM minor version
        int Cores;
    } sSMtoCores;

    sSMtoCores nGpuArchCoresPerSM[] =
    {
        { 0x10,  8 }, // Tesla Generation (SM 1.0) G80 class
        { 0x11,  8 }, // Tesla Generation (SM 1.1) G8x class
        { 0x12,  8 }, // Tesla Generation (SM 1.2) G9x class
        { 0x13,  8 }, // Tesla Generation (SM 1.3) GT200 class
        { 0x20, 32 }, // Fermi Generation (SM 2.0) GF100 class
        { 0x21, 48 }, // Fermi Generation (SM 2.1) GF10x class
        { 0x30, 192}, // Kepler Generation (SM 3.0) GK10x class
        { 0x32, 192}, // Kepler Generation (SM 3.2) GK10x class
        { 0x35, 192}, // Kepler Generation (SM 3.5) GK11x class
        { 0x37, 192}, // Kepler Generation (SM 3.7) GK21x class
        { 0x50, 128}, // Maxwell Generation (SM 5.0) GM10x class
        {   -1, -1 }
    };

    int index = 0;

    while (nGpuArchCoresPerSM[index].SM != -1)
    {
        if (nGpuArchCoresPerSM[index].SM == ((major << 4) + minor))
        {
            return nGpuArchCoresPerSM[index].Cores;
        }

        index++;
    }

    // If we don't find the values, we default use the previous one to run properly
    printf("MapSMtoCores for SM %d.%d is undefined.  Default to use %d Cores/SM\n", major, minor, nGpuArchCoresPerSM[index-1].Cores);
    return nGpuArchCoresPerSM[index-1].Cores;
}

template<typename T> T* CudaUpload(const T* source, int numel)
{
    T* devicePtr;
    cudaMalloc(&devicePtr, sizeof(T)*numel);
    cudaMemcpy(devicePtr, source, sizeof(T)*numel, cudaMemcpyHostToDevice);
    return devicePtr;
}

template<typename T> T* CudaUploadAsync(const T* source, int numel, cudaStream_t stream)
{
    T* devicePtr;
    cudaMalloc(&devicePtr, sizeof(T)*numel);
    cudaMemcpyAsync(devicePtr, source, sizeof(T)*numel, cudaMemcpyHostToDevice, stream);
    return devicePtr;
}

void _CheckCudaErrors(const char* file, int line)
{
	cudaError errorID = cudaGetLastError();
	if(errorID != cudaSuccess)
	{
    std::cout << std::endl;
    std::cout << "CUDA Fatal Error: " << cudaGetErrorString(errorID) << std::endl;
    std::cout << "     AT: " << file << "(" << line << ")" << std::endl;
	}
}
#define CheckCudaErrors() _CheckCudaErrors(__FILE__, __LINE__);

__global__ void cudaAcceptHypothesis(FloatType** model_points, RangeImage image, FloatType* transforms, int num_transforms, int* matches, int gMatchThresh, int gPenaltyThresh)
{
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if(i<num_transforms) acceptHypothesis(i, model_points, image, transforms, matches, gMatchThresh, gPenaltyThresh);
}

void getCUDAThreadConstraint(const int device_id, int &threadConstraint, int &numOfCores, double &power)
{
  // Get some estimate of device power
  // https://devtalk.nvidia.com/default/topic/470848/what-39-s-the-proper-way-to-detect-sp-cuda-cores-count-per-sm-/

  // Set the cuda device
  cudaSetDevice(device_id);

  // Get device properties
  int device;
  cudaDeviceProp deviceProperties;
  cudaGetDevice(&device);
  cudaGetDeviceProperties(&deviceProperties, device);

  // Find out maximum block and thread count
  threadConstraint = deviceProperties.maxThreadsPerBlock;
  if(threadConstraint*kernel_reg_count > deviceProperties.regsPerBlock)
  {
    // if we would use maxThreadsPerBlock we would violate the max register count per block constraint
    threadConstraint = deviceProperties.regsPerBlock / kernel_reg_count;
  }

  // Number of CUDA cores
  numOfCores = _ConvertSMVer2Cores(deviceProperties.major, deviceProperties.minor) * deviceProperties.multiProcessorCount;

  // Arithmetic processing power
  power = numOfCores * deviceProperties.clockRate * 1E-6;

#ifdef OBJ_REC_RANSAC_VERBOSE
  std::cout << "Device ("<<device_id<<") Clock: "" Cores: "<<numOfCores<<" Power: "<<power<<" Max registers per block: " << deviceProperties.regsPerBlock << " -> use " << threadConstraint << " threads." << std::endl;
#endif
}

void cudaAcceptHypotheses(
    FloatType** model_points,
    RangeImage image,
    FloatType* transforms,
    int num_transforms,
    int* matches,
    const int gMatchThresh,
    const int gPenaltyThresh,
    const int device_id,
    boost::mutex &cuda_mutex,
    boost::barrier &cuda_barrier)
{
  // pointers to cuda memory
  cudaStream_t cuda_stream;
  RangeImage image_dev = image;
  std::map<const FloatType*, FloatType*> modelDeviceMap;
  FloatType** device_model_points = new FloatType*[num_transforms];
  FloatType** device_model_points_dev = NULL;
  FloatType* transforms_dev = NULL;
  int* matches_dev = NULL;

  int device;
  cudaDeviceProp deviceProperties;

  {
    boost::mutex::scoped_lock cuda_lock(cuda_mutex);

    // Set the cuda device
    cudaSetDevice(device_id);

    // Create a stream for async processing
    cudaStreamCreate(&cuda_stream);
    CheckCudaErrors();

    // Get device properties
    cudaGetDevice(&device);
    cudaGetDeviceProperties(&deviceProperties, device);

    // Check if we can perform async actions
    if(deviceProperties.asyncEngineCount == 0) {
      std::cerr<<"No support for async actions"<<std::endl;
    }

    image_dev.mPixels = CudaUpload(image.mPixels, image.width*image.height);
    image_dev.mGridSetsP = CudaUpload(image.mGridSetsP, image.width*image.height*3);

    // Copy model data to device memory
    for(int i=0; i<num_transforms; ++i)
    {
      const FloatType* source = model_points[i];
      FloatType* destination = 0;

      // model_points is a list of pointers to model data. Keep a map of already copied models.
      std::map<const FloatType*, FloatType*>::iterator iter = modelDeviceMap.find(source);
      if(iter != modelDeviceMap.end())
      {
        destination = iter->second;
      }
      else
      {
        destination = CudaUpload(source, ORR_NUM_OF_OWN_MODEL_POINTS*3);
        modelDeviceMap[source] = destination;
      }
      device_model_points[i] = destination;
    }

    device_model_points_dev = CudaUpload(device_model_points, num_transforms);
    CheckCudaErrors();

    // Upload transformations to device
    transforms_dev = CudaUpload(transforms, num_transforms*12);
    CheckCudaErrors();

    // Create results structure on device
    cudaMalloc(&matches_dev, sizeof(int)*num_transforms);
    CheckCudaErrors();
  }

  // Find out maximum block and thread count
  int threadConstraint = deviceProperties.maxThreadsPerBlock;
  if(threadConstraint*kernel_reg_count > deviceProperties.regsPerBlock)
  {
    // if we would use maxThreadsPerBlock we would violate the max register count per block constraint
    threadConstraint = deviceProperties.regsPerBlock / kernel_reg_count;
  }

  // call kernel
  int threadsPerBlock = threadConstraint;
  int blocksPerGrid = num_transforms/threadConstraint;

  cudaAcceptHypothesis<<<blocksPerGrid, threadsPerBlock, 0, cuda_stream>>>(device_model_points_dev, image_dev, transforms_dev, num_transforms, matches_dev, gMatchThresh, gPenaltyThresh);

  // Download transformations from GPU
  cudaMemcpyAsync(transforms, transforms_dev, sizeof(FloatType)*num_transforms*12, cudaMemcpyDeviceToHost, cuda_stream);

  // Download match results from GPU
  cudaMemcpyAsync(matches, matches_dev, sizeof(int)*num_transforms, cudaMemcpyDeviceToHost, cuda_stream);

  // Wait for all cuda threads to have dispatched their processing
  cuda_barrier.wait();

  {
    // Wait for the device lock
    boost::mutex::scoped_lock cuda_lock(cuda_mutex);

    // Set the cuda device
    cudaSetDevice(device_id);

    // Block until all memory has been copied
    cudaStreamSynchronize(cuda_stream);

    // Free cuda memory
    cudaFree(image_dev.mPixels);
    cudaFree(image_dev.mGridSetsP);
    cudaFree(device_model_points_dev);
    cudaFree(transforms_dev);
    cudaFree(matches_dev);

    // Free cpu memory
    delete[] device_model_points;

    std::map<const FloatType*, FloatType*>::iterator iter = modelDeviceMap.begin();
    while(iter!=modelDeviceMap.end())
    {
      cudaFree(iter->second);
      ++iter;
    }
  }
}
