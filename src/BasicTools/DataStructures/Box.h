#ifndef _TUM_BOX_H_
#define _TUM_BOX_H_

#include <cstdio>
#include "../LinearAlgebra/Vector.h"

using namespace tum;

namespace tum
{

/** An n-dimensional box = [a_1, b_1] x [a_2, b_2] x ... x [a_n, b_n] */
class Box
{
public:
	Box();
	Box(int dimension);
	Box(const Box& box);
	virtual ~Box();

	inline void alloc(int dimension);
	inline void operator =(const Box& box);
	inline void set(Vector& inf, Vector& sup);
	/** Builds a box with size given by 'size' and middle point given by 'mid'. Both arguments
	  * must have the same dimension like this box. */
	inline void set(double* size, double* mid);
	inline void set(int intervalIndex, double inf, double sup);

	inline void setInf(const Vector& inf);
	inline void setSup(const Vector& sup);
	inline void setInf(int interval, double inf){ mIntervals[interval][0] = inf;}
	inline void setSup(int interval, double sup){ mIntervals[interval][1] = sup;}

	inline double getMiddle(int intervalIndex);
	int getDimension()const{ return mDimension;}

	/** Copies the intervals from src to this object. The method assumes that the
	  * boxes have the same dimension! */
	inline void copyfrom(const Box& src);
	/** Copies the intervals from 'src' to this object. The method assumes that the
	  * 'src' and this box have the same dimension, i.e., 'src' has to be a 'this->mDimension' x 2 matrix. */
	inline void copyfrom(const double** src);
	inline void copyto(double** dst);

	/** The method computes the length of every interval in this object and saves them in dst.
	  * dst must have the same dimension as this object! */
	inline void getDiameter(Vector& dst);
	inline double getDiameter(int intervalIndex){ return mIntervals[intervalIndex][1] - mIntervals[intervalIndex][0];}

	inline void getInf(Vector& dst);
	inline void getSup(Vector& dst);
	double getInf(int interval){ return mIntervals[interval][0];}
	double getSup(int interval){ return mIntervals[interval][1];}

	const double** getIntervals(){ return (const double**)mIntervals;}

	/** Scales this box by 'factor'. The scaling is performed about the center
	  * of the box. */
	inline void scale(double factor);

	inline void intersect(const Box& box);
	inline void join(const Box& box);
	inline bool disjoint(const Box& box);

	/** Returns 'true' iff point lies within this box. */
	inline bool contains(Vector& point);

	/** Print this object to dst. */
	void print(FILE* dst);

protected:
	inline void dealloc();

public:
	double **mIntervals;
	int mDimension;
};

}//namespace tum

//=== inline methods =========================================================================================================

inline void Box::copyfrom(const Box& src)
{
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		mIntervals[i][0] = src.mIntervals[i][0];
		mIntervals[i][1] = src.mIntervals[i][1];
	}
}

//============================================================================================================================

inline void Box::copyfrom(const double** src)
{
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		mIntervals[i][0] = src[i][0];
		mIntervals[i][1] = src[i][1];
	}
}

//============================================================================================================================

inline void Box::copyto(double** dst)
{
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		dst[i][0] = mIntervals[i][0];
		dst[i][1] = mIntervals[i][1];
	}
}

//============================================================================================================================

inline void Box::operator =(const Box& box)
{
	this->alloc(box.mDimension);
	this->copyfrom(box);
}

//============================================================================================================================

inline void Box::alloc(int dimension)
{
	// First free everything that has been reserved
	this->dealloc();
	// 'this->mIntervals' will be a 'dimension' x '2' matrix
	mDimension = dimension;
	mIntervals = new double*[dimension]; // Allocate space for 'dimension' number of intervals (pointers to double).
	for ( int i = 0 ; i < dimension ; ++i )
		mIntervals[i] = new double[2];
}

//============================================================================================================================

inline void Box::dealloc()
{
	if ( mIntervals == NULL )
		return;

	for ( int i = 0 ; i < mDimension ; ++i )
		delete[] mIntervals[i];
	delete[] mIntervals;
	mIntervals = NULL;
	mDimension = 0;
}

//============================================================================================================================

inline void Box::intersect(const Box& box)
{
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		// Lower bound of the current interval
		if ( box.mIntervals[i][0] > this->mIntervals[i][0] )
			this->mIntervals[i][0] = box.mIntervals[i][0];
		// Upper bound of the current interval
		if ( box.mIntervals[i][1] < this->mIntervals[i][1] )
			this->mIntervals[i][1] = box.mIntervals[i][1];
	}
}

//============================================================================================================================

inline void Box::join(const Box& box)
{
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		// Lower bound of the current interval
		if ( box.mIntervals[i][0] < mIntervals[i][0] )
			mIntervals[i][0] = box.mIntervals[i][0];
		// Upper bound of the current interval
		if ( box.mIntervals[i][1] > mIntervals[i][1] )
			mIntervals[i][1] = box.mIntervals[i][1];
	}
}

//============================================================================================================================

inline bool Box::disjoint(const Box& box)
{
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		if ( mIntervals[i][1] < box.mIntervals[i][0] )
			return true;
		if ( mIntervals[i][0] > box.mIntervals[i][1] )
			return true;
	}

	return false;
}

//============================================================================================================================

inline bool Box::contains(Vector& point)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		if ( point.mData[i] < mIntervals[i][0] || point.mData[i] > mIntervals[i][1] )
			return false;

	return true;
}

//============================================================================================================================

inline void Box::scale(double factor)
{
	double mid;
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		mid = this->getMiddle(i);
		mIntervals[i][0] = (mIntervals[i][0] - mid)*factor + mid;
		mIntervals[i][1] = (mIntervals[i][1] - mid)*factor + mid;
	}
}

//============================================================================================================================

inline void Box::getDiameter(Vector& dst)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		dst.mData[i] = mIntervals[i][1] - mIntervals[i][0];
}

//============================================================================================================================

inline void Box::set(Vector& inf, Vector& sup)
{
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		mIntervals[i][0] = inf.mData[i];
		mIntervals[i][1] = sup.mData[i];
	}
}

//============================================================================================================================

inline void Box::set(int intervalIndex, double inf, double sup)
{
	mIntervals[intervalIndex][0] = inf;
	mIntervals[intervalIndex][1] = sup;
}

//============================================================================================================================

inline void Box::set(double* size, double* mid)
{
	double halfsize;
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		halfsize = size[i]/2.0;
		mIntervals[i][0] = mid[i] - halfsize;
		mIntervals[i][1] = mid[i] + halfsize;
	}
}

//============================================================================================================================

inline void Box::setInf(const Vector& inf)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		mIntervals[i][0] = inf.mData[i];
}

//============================================================================================================================


inline void Box::setSup(const Vector& sup)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		mIntervals[i][1] = sup.mData[i];
}

//============================================================================================================================

inline void Box::getInf(Vector& dst)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		dst.mData[i] = mIntervals[i][0];
}

//============================================================================================================================

inline void Box::getSup(Vector& dst)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		dst.mData[i] = mIntervals[i][1];
}

//============================================================================================================================

inline double Box::getMiddle(int intervalIndex)
{
	return 0.5*(mIntervals[intervalIndex][0] + mIntervals[intervalIndex][1]);
}

//============================================================================================================================

#endif /*_TUM_BOX_H_*/
