/*
 * ORRRangeImage2PixelSet.h
 *
 *  Created on: Jun 7, 2010
 *      Author: papazov
 */

#ifndef _ORR_RANGEIMAGE2_PIXEL_SET_H_
#define _ORR_RANGEIMAGE2_PIXEL_SET_H_

#include "../ORROctree/ORROctreeNodeData.h"
#include <BasicTools/ComputationalGeometry/DataStructures/Octree/OctreeNode.h>
#include <set>

using namespace std;

class ORRRangeImage2PixelSet
{
public:
	inline ORRRangeImage2PixelSet(int x, int y);
	inline virtual ~ORRRangeImage2PixelSet();

	void insert(OctreeNode* node){mPixelSet->insert(node);}

	int x()const{ return this->mx;}
	int y()const{ return this->my;}

	const double *p()const{ return mp;}
	void set_p(const double* p){ mp[0] = p[0]; mp[1] = p[1]; mp[2] = p[2];}

	set<OctreeNode*>::iterator begin(){ return mPixelSet->begin();}
	set<OctreeNode*>::iterator end(){ return mPixelSet->end();}

protected:
	set<OctreeNode*, bool(*)(OctreeNode*,OctreeNode*)>* mPixelSet;
	int mx, my;
	double mp[3];
};

//==============================================================================================================================

inline bool fnNodeCompare(OctreeNode* node1, OctreeNode* node2)
{
	return
	((ORROctreeNodeData*)node1->getData())->get3dId()[2]
	<
	((ORROctreeNodeData*)node2->getData())->get3dId()[2];
}

//==============================================================================================================================

inline ORRRangeImage2PixelSet::ORRRangeImage2PixelSet(int x, int y)
{
	mPixelSet = new set<OctreeNode*, bool(*)(OctreeNode*,OctreeNode*)>(fnNodeCompare);
	this->mx = x;
	this->my = y;
}

//==============================================================================================================================

inline ORRRangeImage2PixelSet::~ORRRangeImage2PixelSet()
{
	mPixelSet->clear();
	delete mPixelSet;
}

//==============================================================================================================================

#endif /* _ORR_RANGEIMAGE2_PIXEL_SET_H_ */
