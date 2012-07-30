/*
 * KdTreeNode.cpp
 *
 *  Created on: Dec 15, 2009
 *      Author: papazov
 */

#include "KdTreeNode.h"

using namespace tum;


KdTreeNode::KdTreeNode(int level, BSPTreeNode* parent, const Box& space, BSPTreeNodeData* data)
: BSPTreeNode(level, parent, space, data)
{
	mParent = parent;
	mData = data;
	mChildren[0] = mChildren[1] = NULL;
}

//================================================================================================================================

KdTreeNode::~KdTreeNode()
{
	if ( mChildren[0] ) delete mChildren[0];
	if ( mChildren[1] ) delete mChildren[1];
}

//================================================================================================================================

void KdTreeNode::createChildren()
{
	Box box(mSpace);
	double mid = mSpace.getMiddle(mAxis);

	// First (left) child
	// box.mIntervals[mAxis][0], i.e., the lower bound, does not change
	box.mIntervals[mAxis][1] = mid;
	mChildren[0] = new KdTreeNode(mLevel + 1, this, box, NULL/*data*/);

	// Second (right) child
	box.mIntervals[mAxis][0] = mid;
	box.mIntervals[mAxis][1] = mSpace.mIntervals[mAxis][1];
	mChildren[1] = new KdTreeNode(mLevel + 1, this, box, NULL/*data*/);
}

//================================================================================================================================
