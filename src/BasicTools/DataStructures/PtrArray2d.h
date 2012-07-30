/*
 * Array2d.h
 *
 *  Created on: Sep 29, 2010
 *      Author: papazov
 */

#ifndef _PTR_ARRAY2D_H_
#define _PTR_ARRAY2D_H_

#include <cstdio>

/** A two-dimensional array of pointers. */
template <class T>
class PtrArray2d
{
public:
	/** Creates an 'width' x 'height' 2d array and sets each element to NULL. Note that calling the destructor
	  * of an instance of this class will destroy each element which is not NULL, i.e., the objects to which
	  * 'this->mData' points to will be destroyed! */
	inline PtrArray2d(int width, int height);
	/** Calls the destructor for each non-NULL pointer saved in 'this->mData'. */
	inline virtual ~PtrArray2d();

	int width(){ return mWidth;}
	int height(){ return mHeight;}
	const T*** data(){ return (const T***)mData;}

	inline void setEachElement(T value);

public:
	T ***mData;

protected:
	int mWidth, mHeight;
};

//=== inline methods =================================================================================================

template<class T>
inline PtrArray2d<T>::PtrArray2d(int width, int height)
{
	mWidth = width;
	mHeight = height;
	mData = new T**[width];

	int x, y;
	for ( x = 0 ; x < width ; ++x )
	{
		mData[x] = new T*[height];
		// Set each element to NULL
		for ( y = 0 ; y < height ; ++y )
			mData[x][y] = NULL;
	}
}

//====================================================================================================================

template<class T>
inline PtrArray2d<T>::~PtrArray2d()
{
	int x, y;
	for ( x = 0 ; x < mWidth ; ++x )
	{
		for ( y = 0 ; y < mHeight ; ++y )
			if ( mData[x][y] )
				delete mData[x][y];

		delete[] mData[x];
	}
	delete[] mData;
}

//====================================================================================================================

#endif /* _PTR_ARRAY2D_H_ */
