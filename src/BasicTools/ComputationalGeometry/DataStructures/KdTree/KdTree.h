#ifndef _TUM_KD_TREE_H_
#define _TUM_KD_TREE_H_

#include "../../../DataStructures/Box.h"
#include "KdTreeNode.h"
#include <cstdio>

#include "../BSPTree/BSPTree.h"

namespace tum
{

class KdTree: public BSPTree
{
public:
	KdTree();
	virtual ~KdTree();

	void buildTree(const Box& space, BSPTreeNodeData* rootData);

	KdTreeNode* getRoot(){ return mRoot;}
	void setRoot(KdTreeNode* root){ mRoot = root;}
	int getDimension(){ if ( mRoot ) return mRoot->getDimension(); return 0;}

	bool isInsideTheTree(double* point){ return mRoot->isInside(point);}

	virtual void print(FILE* file, const char* label = NULL){}

protected:
	KdTreeNode* mRoot;
};

}//namespace tum

//=== inline methods ===============================================================================================================

#endif /*_TUM_KD_TREE_H_*/
