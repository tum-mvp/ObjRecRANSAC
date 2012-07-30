/*
 * KdTreeNode.h
 *
 *  Created on: Dec 15, 2009
 *      Author: papazov
 */

#ifndef _TUM_KD_TREE_NODE_H_
#define _TUM_KD_TREE_NODE_H_

#include "../../../DataStructures/Box.h"
#include "../BSPTree/BSPTreeNodeData.h"
#include "../BSPTree/BSPTreeNode.h"
#include <cstdio>

namespace tum
{

class KdTreeNode: public BSPTreeNode
{
public:
	KdTreeNode(int level, BSPTreeNode* parent, const Box& space, BSPTreeNodeData* data);
	virtual ~KdTreeNode();

	BSPTreeNodeData* getDataOfChild(int childId){ return mChildren[childId]->getData();}
	/** This method creates the children of the this node and returns 'true'. If this node already
	  * has children nothing happens and the method returns 'false'. */
	void createChildren();
	/** Avoid using this method unless you know exactly how to create valid children. If you just need
	 * new children for this node use 'this->createChildren'. */
	inline void setChildren(KdTreeNode* left, KdTreeNode* right);
	KdTreeNode** getChildren(){ return mChildren;}
	BSPTreeNode* getChild(int i){ return mChildren[i];}
	bool hasChildren(){ return (bool)mChildren[0];}
	double getLowerBoxBound(int axis){ return mSpace.mIntervals[axis][0];}
	double getUpperBoxBound(int axis){ return mSpace.mIntervals[axis][1];}
	inline int getPreviousAxis();
	inline int getPreviousAxis(int axis);
	int getDimension(){ return mSpace.mDimension;}
	void setParent(KdTreeNode *parent){ mParent = parent;}
	/** Returns 'true' iff 'point' is inside this node. 'point' has to have the same dimension as
	  * this node. */
	inline bool isInside(double* point);

	/** Returns the mean of inf and sup for the current axis. */
	inline double splitAxis(){ return mSpace.getMiddle(mAxis);}

protected:
	KdTreeNode *mChildren[2];
};

}//namespace tum

//=== inline methods =============================================================================================================

inline void tum::KdTreeNode::setChildren(KdTreeNode* left, KdTreeNode* right)
{
	if ( mChildren[0] ) delete mChildren[0];
	if ( mChildren[1] ) delete mChildren[1];
	mChildren[0] = left;
	mChildren[1] = right;
}

//================================================================================================================================

inline bool tum::KdTreeNode::isInside(double* point)
{
	for ( int i = 0 ; i < mSpace.mDimension ; ++i )
		if ( point[i] < mSpace.mIntervals[i][0] || point[i] >= mSpace.mIntervals[i][1] )
			return false;

	return true;
}

//================================================================================================================================

inline int tum::KdTreeNode::getPreviousAxis()
{
	if ( mAxis == 0 )
		return mSpace.mDimension-1;

	return mAxis-1;
}

//================================================================================================================================

inline int tum::KdTreeNode::getPreviousAxis(int axis)
{
	if ( axis == 0 )
		return mSpace.mDimension-1;

	return axis-1;
}

//================================================================================================================================

#endif /*_TUM__KD_TREE_NODE_H_*/
