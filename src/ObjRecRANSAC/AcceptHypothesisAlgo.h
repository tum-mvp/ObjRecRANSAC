#ifndef __ACCEPT_HYPOTHESIS_ALGO_H
#define __ACCEPT_HYPOTHESIS_ALGO_H

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include "DataStructures/DatabaseModelEntry.h"
#include "DataStructures/ThreadInfo.h"
#include "DataStructures/RangeImage/ORRRangeImage2.h"
#include "SVD.h"

typedef FloatType Real;

__device__ __host__ inline void vec_add3(FloatType *v, const FloatType *a)
{
	v[0] += a[0];
	v[1] += a[1];
	v[2] += a[2];
}

__device__ __host__ inline void vec_mult3(FloatType *v, FloatType s)
{
	v[0] *= s;
	v[1] *= s;
	v[2] *= s;
}
__device__ __host__ inline void mat_sub9(FloatType* res, const FloatType* m)
{
	res[0] -= m[0]; res[1] -= m[1]; res[2] -= m[2];
	res[3] -= m[3]; res[4] -= m[4]; res[5] -= m[5];
	res[6] -= m[6]; res[7] -= m[7]; res[8] -= m[8];
}
__device__ __host__ inline void mat_mult3_by_rigid(const FloatType* rigid, const FloatType* p, FloatType* out)
{
	// Rotate and translate
	out[0] = rigid[0]*p[0] + rigid[1]*p[1] + rigid[2]*p[2] + rigid[9];
	out[1] = rigid[3]*p[0] + rigid[4]*p[1] + rigid[5]*p[2] + rigid[10];
	out[2] = rigid[6]*p[0] + rigid[7]*p[1] + rigid[8]*p[2] + rigid[11];
}
__device__ __host__ inline void mat_add_tensor_product_to_mat9(const FloatType* a, const FloatType* b, FloatType* mat)
{
	mat[0] += a[0]*b[0]; mat[1] += a[0]*b[1]; mat[2] += a[0]*b[2];
	mat[3] += a[1]*b[0]; mat[4] += a[1]*b[1]; mat[5] += a[1]*b[2];
	mat[6] += a[2]*b[0]; mat[7] += a[2]*b[1]; mat[8] += a[2]*b[2];
}

//====================================================================================================================

__device__ __host__ inline void vec_mult3(const FloatType *v, FloatType s, FloatType* out)
{
	out[0] = s*v[0];
	out[1] = s*v[1];
	out[2] = s*v[2];
}
__device__ __host__ inline void mat_tensor_product9(const FloatType* a, const FloatType* b, FloatType* out)
{
	out[0] = a[0]*b[0]; out[1] = a[0]*b[1]; out[2] = a[0]*b[2];
	out[3] = a[1]*b[0]; out[4] = a[1]*b[1]; out[5] = a[1]*b[2];
	out[6] = a[2]*b[0]; out[7] = a[2]*b[1]; out[8] = a[2]*b[2];
}


struct FloatType_2 {
    FloatType x, y;
};

class RangeImage
{
public:
    int width, height;
	FloatType mInvPixelSize;
    FloatType mBounds[4];

    FloatType* mGridSetsP;
    FloatType_2* mPixels;

    RangeImage(const ORRRangeImage2* image)
    {
        // pull meta information
        width = image->width();
        height = image->height();
        mInvPixelSize = 1.0 / image->getPixelSize();

        for (int i=0; i<4; ++i) {
            mBounds[i] = image->getBounds()[i];
        }

        // convert pixel data to linear memory
        mPixels = new FloatType_2[width*height];
        for (int x=0; x<width; ++x) {
            for (int y=0; y<height; ++y) {
                if (image->getPixels()[x][y]) {
                    mPixels[x*height + y].x = image->getPixels()[x][y]->x;
                    mPixels[x*height + y].y = image->getPixels()[x][y]->y;
                } else
                {
                    mPixels[x*height + y].x = -1.0;
                    mPixels[x*height + y].y = -1.0;
                }
            }
        }

        // convert grid sets to linear memory
        mGridSetsP = new FloatType[width * height * 3];
        for (int x=0; x<width; ++x) {
            for (int y=0; y<height; ++y) {
                int idx = (x*height*3 + y*3);
                const ORRRangeImage2PixelSet* pixelSet = image->getGridSet(x, y);
                if (pixelSet) {
                    const double* src = pixelSet->p();
                    if(src) {
                        mGridSetsP[idx+0] = src[0];
                        mGridSetsP[idx+1] = src[1];
                        mGridSetsP[idx+2] = src[2];
                    }
                }
            }
        }
    }

    __device__ __host__ const FloatType_2* getSafePixel(FloatType u, FloatType v, int& x, int& y) const
    {
        if ( u < mBounds[0] || u >= mBounds[1] || v < mBounds[2] || v >= mBounds[3] )
            return NULL;

        x = (int)((u-mBounds[0])*mInvPixelSize);
        y = (int)((v-mBounds[2])*mInvPixelSize);

        return &mPixels[x*height + y];
    }

    __device__ __host__ const FloatType* getGridSetP(int x, int y)const
    {
        int idx = (x*height*3 + y*3);
        return &mGridSetsP[idx];
    }
};

__device__ __host__ inline void c_polar_decomposition(const FloatType M[9], FloatType R[9])
{
    Matrix3x3 A(M[0], M[1], M[2], M[3], M[4], M[5], M[6], M[7], M[8]);
    Matrix3x3 Q = PolarDecomposition(A);
    for (int i=0; i<9; ++i) R[i] = Q[i];
}

__device__ __host__ inline void one_icp_iteration(const FloatType* mp, int numOfPoints, const RangeImage& image, FloatType* transform)
{
    FloatType m_0[3];
    FloatType s_0[3];
    FloatType Ncc[9];
    FloatType C[9];

	int k, x, y, match = 0;
	const FloatType_2* pixel;
	const FloatType *sp;
	FloatType out[3];

	// Some initializations
	m_0[0] = m_0[1] = m_0[2] = 0.0;
	s_0[0] = s_0[1] = s_0[2] = 0.0;
	C[0] = C[1] = C[2] = C[3] = C[4] = C[5] = C[6] = C[7] = C[8] = 0.0;

	/* The ICP loop */
	for ( k = 0 ; k < numOfPoints ; ++k, mp += 3 )
	{
		// Transform the model point with the current rigid transform
		mat_mult3_by_rigid(transform, mp, out);

		// Get the pixel the point 'out' lies in
		pixel = image.getSafePixel(out[0], out[1], x, y);
		// Check if we have a valid pixel
		if ( !pixel || (pixel->x == -1.0 && pixel->y == -1.0))
			continue;

		if ( out[2] < pixel->x ) // The transformed model point overshadows a pixel
			continue;
		else if ( out[2] <= pixel->y ) // The point is OK.
		{
			++match;

			// Get the scene point
			sp = image.getGridSetP(x, y);

			// Contribute to the center of mass
			vec_add3(m_0, mp); // We use 'out' only to establish the correspondence
			vec_add3(s_0, sp);
			// Contribute to the covariance matrix
			mat_add_tensor_product_to_mat9(sp, mp, C);
		}
	}

	// We need at least three corresponding point pairs
	if ( match < 3 )
		return;

	// Compute the center of mass for the model
	vec_mult3(m_0, (FloatType)(1.0/(FloatType)match));
	// Compute 'Ncc'
	mat_tensor_product9(s_0, m_0, Ncc);
	// Compute the covariance matrix
	mat_sub9(C, Ncc);
	// Compute the optimal rotation and save it in 'transform'
	//ipp_polar_decomposition(C, transform);
	c_polar_decomposition(C, transform);
    //c_polar_decomposition(C, transform);

	// Compute the center of mass for the scene
	vec_mult3(s_0, (FloatType)(1.0/(FloatType)match));

	// Compute the optimal translation
	transform[9]  = s_0[0] - (transform[0]*m_0[0] + transform[1]*m_0[1] + transform[2]*m_0[2]);
	transform[10] = s_0[1] - (transform[3]*m_0[0] + transform[4]*m_0[1] + transform[5]*m_0[2]);
	transform[11] = s_0[2] - (transform[6]*m_0[0] + transform[7]*m_0[1] + transform[8]*m_0[2]);
}

__device__ __host__ inline void acceptHypothesis(int i, FloatType** model_points, RangeImage image, FloatType* transforms, int* matches, int gMatchThresh, int gPenaltyThresh)
{
    FloatType* transform = &transforms[i*12];
    one_icp_iteration(model_points[i], ORR_NUM_OF_OWN_MODEL_POINTS,
                      image, transform);

    // Some initializations for the second loop (the match/penalty loop)
    FloatType* mp = model_points[i];
    int match = 0, penalty = 0;
    FloatType out[3];
    int x, y;
    const FloatType_2* pixel;

    // The match/penalty loop
    for (int k = 0 ; k < ORR_NUM_OF_OWN_MODEL_POINTS ; ++k, mp += 3 )
    {
        // Transform the model point with the current rigid transform
        mat_mult3_by_rigid(transform, mp, out);

        // Get the pixel the point 'out' lies in
        pixel = image.getSafePixel(out[0], out[1], x, y);
        // Check if we have a valid pixel
        if ( pixel == NULL )
            continue;

        if ( out[2] < pixel->x ) // The transformed model point overshadows a pixel -> penalize it.
            ++penalty;
        else if ( out[2] <= pixel->y ) // The point is OK.
            ++match;
    }

    // Check if we should accept this hypothesis
    if ( match >= gMatchThresh && penalty <= gPenaltyThresh )
    {
        matches[i] = match;
    }
    else
    {
        matches[i] = 0;
    }
}

#endif // ifndef __ACCEPT_HYPOTHESIS_ALGO_H
