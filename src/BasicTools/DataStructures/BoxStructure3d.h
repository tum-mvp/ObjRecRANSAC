/*
 * BoxStructure3d.h
 *
 *  Created on: Mar 29, 2010
 *      Author: papazov
 */

#ifndef BOXSTRUCTURE3D_H_
#define BOXSTRUCTURE3D_H_

#include "../LinearAlgebra/Matrix.h"
#include <cstdio>

using namespace tum;

template <class T>
class BoxStructure3d
{
public:
	inline BoxStructure3d();
	inline virtual ~BoxStructure3d();

	/** Builds a 3d array of boxes each one having size defined by 'boxLengths', The boundaries of the whole structure
	  * are given by 'bounds'. The method computes the right number of boxes such that the structure covers the 3d interval
	  * defined by 'bounds'. The method also allocates memory but does NOT initialize the array in any way. This has to be
	  * done by the user. Use this->getData() and this->getNumberOfBoxes() to initialize the boxes. */
	virtual inline bool build(const double* boxLengths, const double** bounds);

	/** This method deletes the memory used by this object. Note that if the object is used to save pointers, the memory
	  * which is addressed by the pointers is not touched in any way. It is the user's responsibility to free the memory
	  * for each element. */
	virtual inline void clear();

	T*** getData(){ return mData;}
	void getNumberOfBoxes(int* numOfBoxes)const{numOfBoxes[0] = mNumOfBoxes[0]; numOfBoxes[1] = mNumOfBoxes[1]; numOfBoxes[2] = mNumOfBoxes[2];}
	void getBoxLengths(double* lengths)const{lengths[0] = mBoxLengths[0]; lengths[1] = mBoxLengths[1]; lengths[2] = mBoxLengths[2];}
	void getBounds(double** bounds)const{ mBounds.copyto(bounds);}

	const double** getBounds()const{ return (const double**)mBounds.m;}

	inline bool getSafeId(const double* p, int& i, int& j, int& k);

protected:
	T ***mData;
	double mBoxLengths[3];
	Matrix mBounds;
	int mNumOfBoxes[3];
};

//=== inline methods ========================================================================================================

template<class T>
inline BoxStructure3d<T>::BoxStructure3d()
: mBounds(3, 2)
{
	mData = NULL;
	mBoxLengths[0] = mBoxLengths[1] = mBoxLengths[2] = 0.0;
	mNumOfBoxes[0] = mNumOfBoxes[1] = mNumOfBoxes[2] = 0;
	mBounds.m[0][0] = mBounds.m[1][0] = mBounds.m[2][0] =  1.0;
	mBounds.m[0][1] = mBounds.m[1][1] = mBounds.m[2][1] = -1.0;
}

template<class T>
inline BoxStructure3d<T>::~BoxStructure3d()
{
	this->clear();
}

//===========================================================================================================================

template<class T>
inline bool BoxStructure3d<T>::getSafeId(const double* p, int& i, int& j, int& k)
{
	if ( p[0] <  mBounds.m[0][0] || p[1] <  mBounds.m[1][0] || p[2] <  mBounds.m[2][0] )
		return false;

	if ( p[0] >= mBounds.m[0][1] || p[1] >= mBounds.m[1][1] || p[2] >= mBounds.m[2][1] )
		return false;

	i = (int)((p[0] - mBounds.m[0][0])/mBoxLengths[0]);
	j = (int)((p[1] - mBounds.m[1][0])/mBoxLengths[1]);
	k = (int)((p[2] - mBounds.m[2][0])/mBoxLengths[2]);

	return true;
}

//===========================================================================================================================

template<class T>
inline void BoxStructure3d<T>::clear()
{
	if ( mData )
	{
		int x, y;
		// Delete each z-axis
		for ( x = 0 ; x < mNumOfBoxes[0] ; ++x )
			for ( y = 0 ; y < mNumOfBoxes[1] ; ++y )
				delete[] mData[x][y];

		// Delete each y-axis
		for ( x = 0 ; x < mNumOfBoxes[0] ; ++x )
			delete[] mData[x];

		// Delete the rest
		delete[] mData;
		mData = NULL;
	}

	mBoxLengths[0] = mBoxLengths[1] = mBoxLengths[2] = 0.0;
	mNumOfBoxes[0] = mNumOfBoxes[1] = mNumOfBoxes[2] = 0;

	mBounds.m[0][0] = mBounds.m[1][0] = mBounds.m[2][0] =  1.0;
	mBounds.m[0][1] = mBounds.m[1][1] = mBounds.m[2][1] = -1.0;
}

//===========================================================================================================================

template<class T>
inline bool BoxStructure3d<T>::build(const double* boxLengths, const double** bounds)
{
	this->clear();

	mBoxLengths[0] = boxLengths[0];
	mBoxLengths[1] = boxLengths[1];
	mBoxLengths[2] = boxLengths[2];
	mBounds.copyfrom((const double**)bounds);

	mNumOfBoxes[0] = 1 + (int)((bounds[0][1] - bounds[0][0])/mBoxLengths[0]);
	mNumOfBoxes[1] = 1 + (int)((bounds[1][1] - bounds[1][0])/mBoxLengths[1]);
	mNumOfBoxes[2] = 1 + (int)((bounds[2][1] - bounds[2][0])/mBoxLengths[2]);

	double diff;

	// Fit the x-bounds of the structure to the bounds supplied by the user
	diff = bounds[0][0] + mBoxLengths[0]*(double)(mNumOfBoxes[0]) - bounds[0][1];
	if ( diff < 0.0 )
	{
		fprintf(stderr, "ERROR in 'BoxStructure3d<T>::%s()': 'diff' can not be negative.\n", __func__); fflush(stderr);
		return false;
	}
	diff *= 0.5;
	mBounds.m[0][0] -= diff; mBounds.m[0][1] += diff;

	// Fit the y-bounds of the structure to the bounds supplied by the user
	diff = bounds[1][0] + mBoxLengths[1]*(double)(mNumOfBoxes[1]) - bounds[1][1];
	if ( diff < 0.0 )
	{
		fprintf(stderr, "ERROR in 'BoxStructure3d<T>::%s()': 'diff' can not be negative.\n", __func__); fflush(stderr);
		return false;
	}
	diff *= 0.5;
	mBounds.m[1][0] -= diff; mBounds.m[1][1] += diff;

	// Fit the z-bounds of the structure to the bounds supplied by the user
	diff = bounds[2][0] + mBoxLengths[2]*(double)(mNumOfBoxes[2]) - bounds[2][1];
	if ( diff < 0.0 )
	{
		fprintf(stderr, "ERROR in 'BoxStructure3d<T>::%s()': 'diff' can not be negative.\n", __func__); fflush(stderr);
		return false;
	}
	diff *= 0.5;
	mBounds.m[2][0] -= diff; mBounds.m[2][1] += diff;

	// Allocate memory for the data array
	try
	{
		mData = new T**[mNumOfBoxes[0]];
		for ( int x = 0 ; x < mNumOfBoxes[0] ; ++x ) // Along the x-axis
		{
			mData[x] = new T*[mNumOfBoxes[1]]; // Along the y-axis
			for ( int y = 0 ; y < mNumOfBoxes[1] ; ++y )
				mData[x][y] = new T[mNumOfBoxes[2]]; // Elements along the z-axis
		}
	}
	catch ( ... )
	{
		fprintf(stderr, "ERROR in 'BoxStructure3d<T>::%s()': can not allocate memory.\n", __func__);
		fflush(stderr);
		return false;
	}

	return true;
}

//===========================================================================================================================

#endif /* BOXSTRUCTURE3D_H_ */
