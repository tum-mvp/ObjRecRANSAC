/*
 * BSPTree.h
 *
 *  Created on: Mar 28, 2010
 *      Author: papazov
 */

#ifndef BSPTREE_H_
#define BSPTREE_H_

#include "../../../DataStructures/Box.h"
#include "BSPTreeNodeData.h"
#include "BSPTreeNode.h"

class BSPTree
{
public:
	BSPTree();
	virtual ~BSPTree();

	virtual void buildTree(const Box& space, BSPTreeNodeData* rootData) = 0;
	virtual BSPTreeNode* getRoot() = 0;
};

#endif /* BSPTREE_H_ */
