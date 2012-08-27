#include "AcceptHypothesisAlgo.h"
#include <cuda.h>
#include <cuda_runtime.h>

#ifndef CUDA_DEVICE_ID
#define CUDA_DEVICE_ID 0
#endif

template<typename T> T* CudaUpload(const T* source, int numel)
{
    T* devicePtr;
    cudaMalloc(&devicePtr, sizeof(T)*numel);
    cudaMemcpy(devicePtr, source, sizeof(T)*numel, cudaMemcpyHostToDevice);
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

void cudaAcceptHypotheses(FloatType** model_points, RangeImage image, FloatType* transforms, int num_transforms, int* matches, int gMatchThresh, int gPenaltyThresh)
{    
    cudaSetDevice(CUDA_DEVICE_ID);
    RangeImage image_dev = image;
    image_dev.mPixels = CudaUpload(image.mPixels, image.width*image.height);
    image_dev.mGridSetsP = CudaUpload(image.mGridSetsP, image.width*image.height*3);
    
    // Copy model data to device memory
    std::map<const FloatType*, FloatType*> modelDeviceMap;
    
    FloatType** device_model_points = new FloatType*[num_transforms];
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
    FloatType** device_model_points_dev = CudaUpload(device_model_points, num_transforms);
    CheckCudaErrors();
    
    // Upload transformations to device
    FloatType* transforms_dev = CudaUpload(transforms, num_transforms*12);
    CheckCudaErrors();
    
    // Create results structure on device
    int* matches_dev;
    cudaMalloc(&matches_dev, sizeof(int)*num_transforms);
    CheckCudaErrors();

    // Find out maximum block and thread count
	int device;
    cudaDeviceProp deviceProperties;
	// upper bound for the registers used by this kernel. This is printed while compiling the cu file.
    static const int kernel_reg_count = 90; 
    
	cudaGetDevice(&device);
	cudaGetDeviceProperties(&deviceProperties, device);

	int threadConstraint = deviceProperties.maxThreadsPerBlock;
	if(threadConstraint*kernel_reg_count > deviceProperties.regsPerBlock)
	{
        // if we would use maxThreadsPerBlock we would violate the max register count per block constraint
		threadConstraint = deviceProperties.regsPerBlock / kernel_reg_count;
	}
    std::cout << "Max registers per block: " << deviceProperties.regsPerBlock << " -> use " << threadConstraint << " threads." << std::endl;

	// call kernel
	int threadsPerBlock = threadConstraint;
	int blocksPerGrid = num_transforms/threadConstraint;
	cudaAcceptHypothesis<<<blocksPerGrid, threadsPerBlock>>>(device_model_points_dev, image_dev, transforms_dev, num_transforms, matches_dev, gMatchThresh, gPenaltyThresh);
    CheckCudaErrors();
    
    // Download transformations from GPU
    cudaMemcpy(transforms, transforms_dev, sizeof(FloatType)*num_transforms*12, cudaMemcpyDeviceToHost);
    CheckCudaErrors();
    
    // Download match results from GPU
    cudaMemcpy(matches, matches_dev, sizeof(int)*num_transforms, cudaMemcpyDeviceToHost);
    CheckCudaErrors();

    // Free cuda memory
    cudaFree(image_dev.mPixels);
    cudaFree(image_dev.mGridSetsP);
    cudaFree(device_model_points_dev);
    cudaFree(transforms_dev);
    cudaFree(matches_dev);

    std::map<const FloatType*, FloatType*>::iterator iter = modelDeviceMap.begin();
    while(iter!=modelDeviceMap.end())
    {
        cudaFree(iter->second);
        ++iter;
    }
}

