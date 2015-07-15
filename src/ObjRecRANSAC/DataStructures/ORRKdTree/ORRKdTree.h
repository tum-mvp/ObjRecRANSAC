/*
 * KdTreeForPoints.h
 *
 *  Created on: Dec 15, 2009
 *      Author: papazov
 */

#ifndef KDTREEFORPOINTS_H_
#define KDTREEFORPOINTS_H_

#include <BasicTools/ComputationalGeometry/DataStructures/KdTree/KdTree.h>


class ORRKdTree: public KdTree
{
public:
	ORRKdTree();
	virtual ~ORRKdTree();

	/** The leafs of the tree do NOT need to be (hyper)boxes with equal sides. 'leaf_size' should
	  * be an array of length 'dimension' containing the side lengths of the leafs. 'boundBox' has
	  * to be a 'dimension' x 2 matrix. This method creates a tree which will have leafs of size
	  * 'leafSize' and will contain, i.e., will cover 'boundBox'. */
	void buildORRKdTree(int dimension, double* leafSize, double** boundBox);

	/** Extends the tree such that it covers 'bounds', i.e., after calling this method the
	  * bounds of the root will contain 'bounds'. 'bounds' has to be a 'dimension' x 2 matrix. */
	void extendTree(double** bounds);

	/** Creates a leaf which contains 'point', sets 'newLeafWasCreated' to 'true' and returns the new created
	 * leaf. If there is already a leaf which contains the 'point' the method sets 'newLeafWasCreated' to 'false'
	 * and return the existing leaf.
	 * WARNING: make sure that the 'point' lies inside the tree! Use this->isInsideTheTree() if you are not sure. */
	inline KdTreeNode* createLeaf(double* point, bool& newLeafWasCreated);

	int getNumOfTreeLevels(){ return mNumOfTreeLevels;}

	void print(FILE* file, const char* label = NULL);

protected:
	/** Creates a new root and sets the current root as a child of the new one. The axis along which the root will be
	  * extended is the one before the axis of the current root. If, for example, the axis of the current root
	  * is 4, then the root will be extended along axis number 3.
	  * The direction of extension is given by 'extdir'.
	  * If 'extdir[i]' == -1 the root will be extended along the i-th axis to the left (the lower bound will be changed),
	  * i.e., the current root will be the right child of the new root.
	  * If 'extdir[i]' ==  1 the root will be extended along the i-th axis to the right (the upper bound will be changed)
	  * and the current root will be the left child of the new root. */
	void extendTreeByOneLevel(char* extdir);

private:
	int mNumOfTreeLevels;
};

//=== inline methods ===================================================================================================================

inline KdTreeNode* ORRKdTree::createLeaf(double* point, bool& newLeafWasCreated)
{
	int i, d, dim = this->getDimension();
	KdTreeNode* node = mRoot;

	for ( i = 0 ; i < mNumOfTreeLevels ; ++i )
	{
		for ( d = 0 ; d < dim ; ++d )
		{
			// If 'node' already has children nothing happens
			newLeafWasCreated = !(node->hasChildren());
			node->createChildren();

			// Check the current coordinate of the 'point'
			if ( point[node->getAxis()] < ((KdTreeNode*)node->getChild(0))->getUpperBoxBound(node->getAxis()) )
				node = (KdTreeNode*)node->getChild(0);
			else
				node = (KdTreeNode*)node->getChild(1);
		}
	}

	return node;
}

//======================================================================================================================================

#endif /* KDTREEFORPOINTS_H_ */
