/*
 * ORRRangeImage2.h
 *
 *  Created on: Jun 4, 2010
 *      Author: papazov
 */

#ifndef _ORR_RANGE_IMAGE_2_H_
#define _ORR_RANGE_IMAGE_2_H_

#include "../../ORRBasicTypes.h"
#include "ORRRangeImage2PixelSet.h"
#include "../ORROctree/ORROctree.h"
#include <list>

using namespace std;

class ORRRangeImage2
{
public:
	ORRRangeImage2();
	virtual ~ORRRangeImage2();

	void buildFromOctree(ORROctree* octree, double front_eps, double back_eps);

	inline void getPixelCoordinates(const double* p, int& x, int& y);
	inline double_2* getSafePixel(const double* p);
	inline const double_2* getSafePixel(double u, double v, int& x, int& y) const;
	inline void getPixelBounds(double* bounds, int x, int y) const;
	void getNumberOfPixels(int& x, int& y){x = mNumOfPixelsX; y = mNumOfPixelsY;}
	int width()const{ return mNumOfPixelsX;}
	int height()const{ return mNumOfPixelsY;}
	int getNumberOfPixels()const{ return mNumOfPixels;}
	const double_2*** getPixels()const{ return (const double_2***)mPixels;}
	const double_3* getNormal(int x, int y)const{ return mNormals[x][y];}
	double getPixelSize()const{ return mPixelSize;}
	const double* getBounds()const{ return mBounds;}
	const list<OctreeNode*>*** getOctreeNodes()const{ return (const list<OctreeNode*>***)mOctreeNodes;}
	const list<ORRRangeImage2PixelSet*>& getFullGridSets()const{ return mFullSets;}
	const ORRRangeImage2PixelSet* getGridSet(int x, int y)const{ return mGridSets[x][y];}

	int getLinearId(int x, int y)const{ return y*mNumOfPixelsX + x;}

	list<int>*** getShapesGrid(){ return mShapesGrid;}

protected:
	void projectOctree(ORROctree* octree);
	void clear();
	void clearGridSets();

protected:
	int mNumOfPixelsX, mNumOfPixelsY, mNumOfPixels;
	double_2*** mPixels;
	double_3*** mNormals;
	double mPixelSize, mInvPixelSize, mExtentX, mExtentY, mBounds[4];

	/** There is a set<OcreeNode*> for each full pixel in this range image.
	  * The octrees are sorted according to their z-coordinate (the integer
	  * coordinate). */
	ORRRangeImage2PixelSet*** mGridSets;
	list<ORRRangeImage2PixelSet*> mFullSets;
	list<int>*** mShapesGrid;
	list<OctreeNode*>*** mOctreeNodes;
};

//=== inline methods ==============================================================================================

inline void ORRRangeImage2::getPixelCoordinates(const double* p, int& x, int& y)
{
	x = (int)((p[0]-mBounds[0])/mPixelSize);
	y = (int)((p[1]-mBounds[2])/mPixelSize);
}

//=================================================================================================================

inline double_2* ORRRangeImage2::getSafePixel(const double* p)
{
	int x = (int)((p[0]-mBounds[0])/mPixelSize);
	int y = (int)((p[1]-mBounds[2])/mPixelSize);

	// Check if we are out of bounds
	if ( x < 0 || y < 0 || x >= mNumOfPixelsX || y >= mNumOfPixelsY )
		return NULL;

	return mPixels[x][y];
}

//=================================================================================================================

inline const double_2* ORRRangeImage2::getSafePixel(double u, double v, int& x, int& y) const
{
	if ( u < mBounds[0] || u >= mBounds[1] || v < mBounds[2] || v >= mBounds[3] )
		return NULL;

	x = (int)((u-mBounds[0])*mInvPixelSize);
	y = (int)((v-mBounds[2])*mInvPixelSize);

	return mPixels[x][y];
}

//=================================================================================================================

inline void ORRRangeImage2::getPixelBounds(double* bounds, int x, int y) const
{
	bounds[0] = mBounds[0] + (double)x*mPixelSize;
	bounds[1] =  bounds[0] + mPixelSize;
	bounds[2] = mBounds[2] + (double)y*mPixelSize;
	bounds[3] =  bounds[2] + mPixelSize;
	bounds[4] = mPixels[x][y]->x;
	bounds[5] = mPixels[x][y]->y;
}

//=================================================================================================================

#endif // _ORR_RANGE_IMAGE_2_H_
