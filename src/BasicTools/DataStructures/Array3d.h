/*
 * Array3d.h
 *
 *  Created on: Jun 11, 2010
 *      Author: papazov
 */

#ifndef ARRAY3D_H_
#define ARRAY3D_H_

template <class T>
class Array3d
{
public:
	inline Array3d(int width, int height, int depth);
	inline virtual ~Array3d();

	int width(){ return mWidth;}
	int height(){ return mHeight;}
	int depth(){ return mDepth;}
	const T*** data(){ return (const T***)mData;}

	inline void setEachElement(T value);

public:
	T ***mData;

protected:
	int mWidth, mHeight, mDepth;
};

//=== inline methods =================================================================================================

template<class T>
inline Array3d<T>::Array3d(int width, int height, int depth)
{
	mWidth = width;
	mHeight = height;
	mDepth = depth;

	mData = new T**[width];
	for ( int x = 0 ; x < width ; ++x ) // Along the x-axis
	{
		mData[x] = new T*[height]; // Along the y-axis
		for ( int y = 0 ; y < height ; ++y )
			mData[x][y] = new T[depth]; // Elements along the z-axis
	}
}

//====================================================================================================================

template<class T>
inline Array3d<T>::~Array3d()
{
	int x, y;
	// Delete each z-axis
	for ( x = 0 ; x < mWidth ; ++x )
		for ( y = 0 ; y < mHeight ; ++y )
			delete[] mData[x][y];

	// Delete each y-axis
	for ( x = 0 ; x < mWidth ; ++x )
		delete[] mData[x];

	// Delete the rest
	delete[] mData;
}

//====================================================================================================================

template<class T>
inline void Array3d<T>::setEachElement(T value)
{
	int x, y, z;
	for ( x = 0 ; x < mWidth ; ++x )
		for ( y = 0 ; y < mHeight ; ++y )
			for ( z = 0 ; z < mDepth ; ++z )
				mData[x][y][z] = value;
}

//====================================================================================================================

#endif /* ARRAY3D_H_ */
