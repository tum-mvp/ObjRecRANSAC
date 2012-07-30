/*
 * BSPTreeNode.h
 *
 *  Created on: Mar 30, 2010
 *      Author: papazov
 */

#ifndef BSPTREENODE_H_
#define BSPTREENODE_H_

#include "../../../DataStructures/Box.h"
#include "BSPTreeNodeData.h"

class BSPTreeNode
{
public:
	BSPTreeNode(int level, BSPTreeNode* parent, const Box& space, BSPTreeNodeData* data);
	virtual ~BSPTreeNode();

	virtual BSPTreeNode* getChild(int id) = 0;
	virtual bool hasChildren() = 0;
	virtual void createChildren() = 0;
	virtual BSPTreeNodeData* getDataOfChild(int id) = 0;

	/** Returns the middle of the current axis of this node. It is not always the mean of inf and sup
	  * but depends on the space this node is representing. */
	virtual double splitAxis() = 0;

	BSPTreeNode* getParent(){ return mParent;}
	BSPTreeNodeData* getData(){ return mData;}
	void setData(BSPTreeNodeData* data){ if ( mData ) delete mData; mData = data;}
	Box& getSpace(){ return mSpace;}
	int getLevel(){ return mLevel;}
	int getAxis(){ return mAxis;}

protected:
	int mLevel, mAxis;
	BSPTreeNode* mParent;
	Box mSpace;
	BSPTreeNodeData* mData;
};

#endif /* BSPTREENODE_H_ */
