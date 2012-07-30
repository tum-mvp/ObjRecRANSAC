#include "KdTree.h"

using namespace tum;


KdTree::KdTree()
{
	mRoot = NULL;
}

KdTree::~KdTree()
{
	if ( mRoot )
		delete mRoot;
}

//==================================================================================================================================

void KdTree::buildTree(const Box& space, BSPTreeNodeData* rootData)
{
	if ( mRoot )
		delete mRoot;

	mRoot = new KdTreeNode(0/*level*/, NULL/*parent*/, space, rootData);
}

//==================================================================================================================================
