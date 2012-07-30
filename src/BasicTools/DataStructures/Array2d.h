/*
 * Array2d.h
 *
 *  Created on: Sep 29, 2010
 *      Author: papazov
 */

#ifndef ARRAY2D_H_
#define ARRAY2D_H_

template <class T>
class Array2d
{
public:
	inline Array2d(int width, int height);
	inline Array2d(int width, int height, T default_value);
	inline virtual ~Array2d();

	int width(){ return mWidth;}
	int height(){ return mHeight;}
	const T** data(){ return (const T**)mData;}

	inline void setEachElement(T value);

public:
	T **mData;

protected:
	int mWidth, mHeight;
};

//=== inline methods =================================================================================================

template<class T>
inline Array2d<T>::Array2d(int width, int height)
{
	mWidth = width;
	mHeight = height;

	mData = new T*[width];
	for ( int x = 0 ; x < width ; ++x ) // Along the x-axis
		mData[x] = new T[height]; // Along the y-axis
}

//====================================================================================================================

template<class T>
inline Array2d<T>::Array2d(int width, int height, T default_value)
{
	mWidth = width;
	mHeight = height;

	mData = new T*[width];
	for ( int x = 0 ; x < width ; ++x ) // Along the x-axis
	{
		mData[x] = new T[height]; // Along the y-axis
		for ( int y = 0 ; y < height ; ++y )
			mData[x][y] = default_value;
	}
}

//====================================================================================================================

template<class T>
inline Array2d<T>::~Array2d()
{
	// Delete each y-axis
	for ( int x = 0 ; x < mWidth ; ++x )
		delete[] mData[x];

	// Delete the rest
	delete[] mData;
}

//====================================================================================================================

template<class T>
inline void Array2d<T>::setEachElement(T value)
{
	int x, y;
	for ( x = 0 ; x < mWidth ; ++x )
		for ( y = 0 ; y < mHeight ; ++y )
			mData[x][y] = value;
}

//====================================================================================================================

#endif /* ARRAY2D_H_ */
