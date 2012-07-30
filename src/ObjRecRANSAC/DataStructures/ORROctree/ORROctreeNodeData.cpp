#include "ORROctreeNodeData.h"

ORROctreeNodeData::ORROctreeNodeData()
{
	mNormal = NULL;
	mNumOfPoints = 0;
	mPoint[0] = mPoint[1] = mPoint[2] = 0.0;
	mOwnPointId = -1;
	m3dId[0] = m3dId[1] = m3dId[2] = -1;
}

ORROctreeNodeData::~ORROctreeNodeData()
{
	if ( mNormal ) delete[] mNormal;
	mPointIds.clear();
}
