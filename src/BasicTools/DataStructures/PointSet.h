/*
 * PointSet.h
 *
 *  Created on: Aug 18, 2010
 *      Author: papazov
 */

#ifndef POINTSET_H_
#define POINTSET_H_

#include <cstdio>
#include <cstring>
#include <BasicToolsL1/Vector.h>

using namespace tum;

template <class T>
class PointSet
{
public:
	inline PointSet();
	inline PointSet(int numberOfPoints);
	inline virtual ~PointSet();

	virtual inline void alloc(int numberOfPoints);
	virtual inline void clear();

	inline void copyfrom(const PointSet<T>* src);

	/** Performs a memset on the array with the point coordinates. */
	void memsetPoints(int value){ memset(mPoints, value, 3*mNumOfPoints*sizeof(T));}

	int getNumberOfPoints()const{ return mNumOfPoints;}
	int getNumberOfBytes()const{ return 3*mNumOfPoints*sizeof(T);}
	const T* getPoints_const()const{ return mPoints;}
	T* getPoints(){ return mPoints;}
	const T* getBounds()const{ return mBounds;}
	inline void setBounds(const T* b);
	inline void getBounds(T* b)const;
	const T* getPoint(int id)const{ return mPoints + 3*id;}
	void setPoint(int id, const double* p){ vec_copy3(p, mPoints + 3*id);}

	inline void addToPoint(int id, const T* u);

protected:
	T* mPoints;
	int mNumOfPoints;
	double mBounds[6];
};

//=== inline methods =======================================================================================

template <class T>
inline PointSet<T>::PointSet()
{
	mPoints = NULL;
	mNumOfPoints = 0;
}

//==========================================================================================================

template <class T>
inline PointSet<T>::PointSet(int numberOfPoints)
{
	mPoints = new T[3*numberOfPoints];
	mNumOfPoints = numberOfPoints;
}

//==========================================================================================================

template <class T>
inline PointSet<T>::~PointSet()
{
	PointSet::clear();
}

//==========================================================================================================

template <class T>
inline void PointSet<T>::clear()
{
	if ( mPoints )
	{
		delete[] mPoints;
		mPoints = NULL;
	}
	mNumOfPoints = 0;
}

//==========================================================================================================

template <class T>
inline void PointSet<T>::alloc(int numberOfPoints)
{
	PointSet::clear();
	mPoints = new T[3*numberOfPoints];
	mNumOfPoints = numberOfPoints;
}

//==========================================================================================================

template <class T>
inline void PointSet<T>::copyfrom(const PointSet<T>* src)
{
	PointSet<T>::alloc(src->getNumberOfPoints());
	memcpy(this->mPoints, src->getPoints_const(), 3*mNumOfPoints*sizeof(T));
	src->getBounds(mBounds);
}

//==========================================================================================================

template <class T>
inline void PointSet<T>::getBounds(T* b) const
{
	b[0] = mBounds[0];
	b[1] = mBounds[1];
	b[2] = mBounds[2];
	b[3] = mBounds[3];
	b[4] = mBounds[4];
	b[5] = mBounds[5];
}

//==========================================================================================================

template <class T>
inline void PointSet<T>::setBounds(const T* b)
{
	mBounds[0] = b[0];
	mBounds[1] = b[1];
	mBounds[2] = b[2];
	mBounds[3] = b[3];
	mBounds[4] = b[4];
	mBounds[5] = b[5];
}

//==========================================================================================================

template <class T>
inline void PointSet<T>::addToPoint(int id, const T* u)
{
	double* p = mPoints + 3*id;
	p[0] += u[0];
	p[1] += u[1];
	p[2] += u[2];
}

//==========================================================================================================

#endif /* POINTSET_H_ */
