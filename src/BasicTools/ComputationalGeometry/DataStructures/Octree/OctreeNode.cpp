#include "OctreeNode.h"
#include <cmath>
#include <cstdio>

using namespace tum;

OctreeNode::OctreeNode()
{
	mChildren = NULL;
	mData = NULL;
	mParent = NULL;
	mVoxelRadius = -1.0;
	mFullDescendantsFlags = 0x00;
	mTmpFlag = 0;
	mChildNr = 255;
	mNeighbours = NULL;
	mNumOfNeighbours = 0;
	mNeighCheck = 0;
	mFullLeafId = -1;
}

OctreeNode::~OctreeNode()
{
	if ( mData )
		delete mData;
	if ( mChildren )
		delete[] mChildren;
	if ( mNeighbours )
		delete[] mNeighbours;
}

//=================================================================================================================================

void OctreeNode::destroyChildren()
{
	if ( mChildren )
	{
		delete[] mChildren;
		mChildren = NULL;
	}
}

//=================================================================================================================================

void OctreeNode::destroyData()
{
	if ( mData )
	{
		delete mData;
		mData = NULL;
	}
}

//=================================================================================================================================

void OctreeNode::createChildren()
{
	if ( mChildren )
		return;

	double bounds[6], center[3], childside = (mBounds[1]-mBounds[0])/2.0;
	mChildren = new OctreeNode[8];

	// Compute bounds and center for child 0, i.e., for (0,0,0)
	bounds[0] = mBounds[0]; bounds[1] = mCenter[0];
	bounds[2] = mBounds[2]; bounds[3] = mCenter[1];
	bounds[4] = mBounds[4]; bounds[5] = mCenter[2];
	this->computeMiddlePoint(bounds, center);
	// Save the results
	mChildren[0].setBounds(bounds);
	mChildren[0].setCenter(center);
	mChildren[0].setParent(this);
	mChildren[0].mChildNr = 0;

	// Compute bounds and center for child 1, i.e., for (0,0,1)
	bounds[4] = mCenter[2]; bounds[5] = mBounds[5];
	// Update the center
	center[2] += childside;
	// Save the results
	mChildren[1].setBounds(bounds);
	mChildren[1].setCenter(center);
	mChildren[1].setParent(this);
	mChildren[1].mChildNr = 1;

	// Compute bounds and center for child 3, i.e., for (0,1,1)
	bounds[2] = mCenter[1]; bounds[3] = mBounds[3];
	// Update the center
	center[1] += childside;
	// Save the results
	mChildren[3].setBounds(bounds);
	mChildren[3].setCenter(center);
	mChildren[3].setParent(this);
	mChildren[3].mChildNr = 3;

	// Compute bounds and center for child 2, i.e., for (0,1,0)
	bounds[4] = mBounds[4]; bounds[5] = mCenter[2];
	// Update the center
	center[2] -= childside;
	// Save the results
	mChildren[2].setBounds(bounds);
	mChildren[2].setCenter(center);
	mChildren[2].setParent(this);
	mChildren[2].mChildNr = 2;

	// Compute bounds and center for child 6, i.e., for (1,1,0)
	bounds[0] = mCenter[0]; bounds[1] = mBounds[1];
	// Update the center
	center[0] += childside;
	// Save the results
	mChildren[6].setBounds(bounds);
	mChildren[6].setCenter(center);
	mChildren[6].setParent(this);
	mChildren[6].mChildNr = 6;

	// Compute bounds and center for child 7, i.e., for (1,1,1)
	bounds[4] = mCenter[2]; bounds[5] = mBounds[5];
	// Update the center
	center[2] += childside;
	// Save the results
	mChildren[7].setBounds(bounds);
	mChildren[7].setCenter(center);
	mChildren[7].setParent(this);
	mChildren[7].mChildNr = 7;

	// Compute bounds and center for child 5, i.e., for (1,0,1)
	bounds[2] = mBounds[2]; bounds[3] = mCenter[1];
	// Update the center
	center[1] -= childside;
	// Save the results
	mChildren[5].setBounds(bounds);
	mChildren[5].setCenter(center);
	mChildren[5].setParent(this);
	mChildren[5].mChildNr = 5;

	// Compute bounds and center for child 4, i.e., for (1,0,0)
	bounds[4] = mBounds[4]; bounds[5] = mCenter[2];
	// Update the center
	center[2] -= childside;
	// Save the results
	mChildren[4].setBounds(bounds);
	mChildren[4].setCenter(center);
	mChildren[4].setParent(this);
	mChildren[4].mChildNr = 4;
}

//=================================================================================================================================

void OctreeNode::setData(OctreeNodeData* data)
{
	if ( mData )
		delete mData;

	mData = data;
}

//=================================================================================================================================

void OctreeNode::setBounds(const double* bounds)
{
	mBounds[0] = bounds[0];
	mBounds[1] = bounds[1];
	mBounds[2] = bounds[2];
	mBounds[3] = bounds[3];
	mBounds[4] = bounds[4];
	mBounds[5] = bounds[5];

	double size = this->getNodeSize();
	mVoxelRadius = sqrt(3.0)*(size/2.0);
}

//=================================================================================================================================

void OctreeNode::setCenter(const double* center)
{
	mCenter[0] = center[0];
	mCenter[1] = center[1];
	mCenter[2] = center[2];
}

//=================================================================================================================================

void OctreeNode::computeMiddlePoint(const double* bounds, double* mp)
{
	mp[0] = (bounds[0] + bounds[1])/2.0;
	mp[1] = (bounds[2] + bounds[3])/2.0;
	mp[2] = (bounds[4] + bounds[5])/2.0;
}

//=================================================================================================================================
