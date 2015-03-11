

#include "AcceptHypothesisAlgo.h"

#include <boost/thread/barrier.hpp>

#ifdef USE_CUDA

extern void cudaAcceptHypotheses(
    FloatType** model_points,
    RangeImage image,
    FloatType* transforms,
    int num_transforms,
    int* matches,
    const int gMatchThresh,
    const int gPenaltyThresh,
    const int device_id,
    boost::mutex &cuda_mutex,
    boost::barrier &cuda_barrier);

#endif

void cpuAcceptHypotheses(FloatType** model_points, RangeImage image, FloatType* transforms, int num_transforms, int* matches, int gMatchThresh, int gPenaltyThresh)
{
	for (int i = 0 ; i < num_transforms ; ++i )
	{
		acceptHypothesis(i, model_points, image, transforms, matches, gMatchThresh, gPenaltyThresh);
	}
}

void acceptCUDA(
    ThreadInfo *info,
    int gMatchThresh,
    int gPenaltyThresh,
    const ORRRangeImage2 *gImage,
    int device_id,
    boost::mutex &cuda_mutex,
    boost::barrier &cuda_barrier)
{
#ifdef OBJ_REC_RANSAC_VERBOSE
    std::cerr<<"Accepting hypotheses with CUDA device "<<device_id<<std::endl;
#endif

    int num_transforms = info->num_transforms;

    // Convert range image into cuda supported structure
    RangeImage image(gImage);

    // Convert model data to floating point type
    std::map<const double*, FloatType*> modelMap;
    FloatType** model_points = new FloatType*[num_transforms];
    for(int i=0; i<num_transforms; ++i)
    {
        const double* source = info->model_points[i];
        FloatType* destination = 0;

        // model_points is a list of pointers to model data. Keep a map of already copied models.
        std::map<const double*, FloatType*>::iterator iter = modelMap.find(source);
        if(iter != modelMap.end())
        {
            destination = iter->second;
        }
        else
        {
            destination = new FloatType[ORR_NUM_OF_OWN_MODEL_POINTS*3];
            for(int i=0; i<ORR_NUM_OF_OWN_MODEL_POINTS*3; ++i) destination[i] = source[i];
            modelMap[source] = destination;
        }
        model_points[i] = destination;
    }

    // Convert input transformations to right floating point type
    FloatType* transforms = new FloatType[num_transforms*12];
    for(int i=0; i<num_transforms*12; ++i) transforms[i] = info->transforms[i];

    // create results structure
    int* matches = new int[num_transforms];

#ifdef CUDA_TEST_CPU
    cpuAcceptHypotheses(
        model_points, image, transforms, num_transforms, matches,
        gMatchThresh, gPenaltyThresh,
        );
#else
    cudaAcceptHypotheses(
        model_points, image, transforms, num_transforms, matches,
        gMatchThresh, gPenaltyThresh, device_id, cuda_mutex, cuda_barrier);
#endif

    // Convert resulting transformations back to objRANSAC float type
    for (int i = 0 ; i < num_transforms*12 ; ++i ) info->transforms[i] = transforms[i];

    // Find best match for each pair from all transforms
    for (int i = 0 ; i < num_transforms ; ++i )
    {
        int pair_id = info->pair_ids[i];
        if ( matches[i] > info->pair_result[pair_id].match )
        {
            info->pair_result[pair_id].match = matches[i];
            info->pair_result[pair_id].model_entry = info->model_entries[i];
            info->pair_result[pair_id].transform = &info->transforms[i*12];
        }
    }


    delete[] (image.mPixels);
    delete[] (image.mGridSetsP);
    delete[] (model_points);
    delete[] (transforms);
    delete[] (matches);

    std::map<const double*, FloatType*>::iterator iter = modelMap.begin();
    while(iter!=modelMap.end())
    {
        delete[] (iter->second);
        ++iter;
    }
}

