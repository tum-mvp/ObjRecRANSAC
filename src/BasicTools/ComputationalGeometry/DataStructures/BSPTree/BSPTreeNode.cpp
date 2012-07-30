/*
 * BSPTreeNode.cpp
 *
 *  Created on: Mar 30, 2010
 *      Author: papazov
 */

#include "BSPTreeNode.h"

BSPTreeNode::BSPTreeNode(int level, BSPTreeNode* parent, const Box& space, BSPTreeNodeData* data)
: mSpace(space)
{
	mAxis = level % space.getDimension();
	mLevel = level;
	mParent = parent;
	mData = data;
}

BSPTreeNode::~BSPTreeNode()
{
	if ( mData ) delete mData;
}
