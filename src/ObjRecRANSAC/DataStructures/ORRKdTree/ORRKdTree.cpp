/*
 * KdTreeForPoints.cpp
 *
 *  Created on: Dec 15, 2009
 *      Author: papazov
 */

#include "ORRKdTree.h"
#include <BasicTools/DataStructures/Box.h>
#include <BasicTools/Aux/NumberUtils.h>

ORRKdTree::ORRKdTree()
{
	mNumOfTreeLevels = 0;
}

ORRKdTree::~ORRKdTree()
{
}

//===============================================================================================================================

void ORRKdTree::print(FILE* file, const char* label)
{
	if ( label ) printf("----------------------- %s -----------------------\n", label);
	else         printf("----------------------- ORRKdTree info -----------------------\n");

	if ( mRoot )
	{
		printf("Dimension = %i\n", this->getDimension());
		printf("Number Of tree levels = %i\n", mNumOfTreeLevels);
		printf("Tree bounds =\n");
		for ( int i = 0 ; i < mRoot->getSpace().getDimension() ; ++i )
			printf("[%9.3lf, %9.3lf]\n", mRoot->getSpace().mIntervals[i][0], mRoot->getSpace().mIntervals[i][1]);
	}
	printf("--------------------------------------------------------------\n");
}

//===============================================================================================================================

void ORRKdTree::extendTree(double** bounds)
{
	int i, axis, dim = this->getDimension();
	double upperDiff, lowerDiff;
	bool extend;
	char altdir = 1, *extdir = new char[dim];

	do
	{
		axis = mRoot->getPreviousAxis();
		extend = false;
		for ( i = 0 ; i < dim ; ++i )
		{
			upperDiff = bounds[axis][1] - mRoot->getUpperBoxBound(axis);
			lowerDiff = mRoot->getLowerBoxBound(axis) - bounds[axis][0];

			if ( upperDiff >= 0 )
			{
				if ( lowerDiff > 0 )
				{
					if ( lowerDiff < upperDiff ) extdir[axis] = -1;
					else extdir[axis] = 1;
				}
				else extdir[axis] = 1;
				extend = true;
			}
			else if ( lowerDiff > 0 )
			{
				extdir[axis] = -1;
				extend = true;
			}
			else
			{
				extdir[axis] = altdir;
				altdir *= -1;
			}

			// Get the next axis
			axis = mRoot->getPreviousAxis(axis);
		}

		// Extend the root
		if ( extend )
			this->extendTreeByOneLevel(extdir);
	}
	while ( extend );

	// Clean up
	delete[] extdir;
}

//===============================================================================================================================

void ORRKdTree::extendTreeByOneLevel(char* extdir)
{
	int i, newaxis, dim = this->getDimension();
	Box newbox(dim);
	KdTreeNode *newroot = NULL, *newchild = NULL;
	double boxsize;

	for ( i = 0 ; i < dim ; ++i )
	{
		newaxis = mRoot->getPreviousAxis();
		newbox.copyfrom(mRoot->getSpace());
		boxsize = newbox.getDiameter(newaxis);

		if ( extdir[newaxis] == -1 )
		{
			// Fix the box of the new root and create the root. Extend the lower bound of the box
			newbox.mIntervals[newaxis][0] -= boxsize;
			newroot = new KdTreeNode(newaxis, NULL/*no parent*/, newbox, NULL);
			// Fix the box of the new child and create the new child
			newbox.mIntervals[newaxis][1] = mRoot->getLowerBoxBound(newaxis);
			newchild = new KdTreeNode(mRoot->getAxis(), newroot/*the parent*/, newbox, NULL);
			// Now fix the children of the new root
			newroot->setChildren(newchild, mRoot);
		}
		else if ( extdir[newaxis] == 1 )
		{
			// Fix the box of the new root and create the root. Extend the upper bound of the box
			newbox.mIntervals[newaxis][1] += boxsize;
			newroot = new KdTreeNode(newaxis, NULL/*no parent*/, newbox, NULL);
			// Fix the box of the new child and create the new child
			newbox.mIntervals[newaxis][0] = mRoot->getUpperBoxBound(newaxis);
			newchild = new KdTreeNode(mRoot->getAxis(), newroot/*the parent*/, newbox, NULL);
			// Now fix the children of the new root
			newroot->setChildren(mRoot, newchild);
		}
		else
		{
			fprintf(stderr, "ERROR in 'ORRKdTree::%s()': extension direction for the %i-th axis is %i! Can not extend the tree!\n",
					__func__, newaxis, extdir[newaxis]);
			fflush(stdout);
			return;
		}

		// Set the new root as the parent of the old one
		mRoot->setParent(newroot);
		// Save the new root
		mRoot = newroot;
	}

	// Now we have one level more
	++mNumOfTreeLevels;
}

//===============================================================================================================================

void ORRKdTree::buildORRKdTree(int dimension, double* leafSize, double** boundBox)
{
	int i, levels;
	double *treeExtent = new double[dimension], *bbSize = new double[dimension], *bbMid = new double[dimension];

	for ( i = 0 ; i < dimension ; ++i )
	{
		if ( leafSize[i] <= 0.0 )
		{
			fprintf(stderr, "ERROR in 'ORRKdTree::%s()': leaf size must be positive! Building the KdTree failed!\n", __func__);
			fflush(stderr);
			// Clean up
			delete[] treeExtent; delete[] bbSize; delete[] bbMid;
			return;
		}
		treeExtent[i] = leafSize[i];
		bbSize[i] = boundBox[i][1] - boundBox[i][0];
		bbMid[i] = (boundBox[i][1] + boundBox[i][0])/2.0;
	}

	mNumOfTreeLevels = 0;

	for ( i = 0 ; i < dimension ; ++i )
	{
		levels = 0;
		while ( treeExtent[i] < bbSize[i] )
		{
			treeExtent[i] *= 2.0;
			++levels;
		}
		if ( levels > mNumOfTreeLevels )
			mNumOfTreeLevels = levels;
	}

	double powOf2 = (double)NumberUtils::powerOf2(mNumOfTreeLevels);
	for ( i = 0 ; i < dimension ; ++i )
		treeExtent[i] = leafSize[i]*powOf2;

	// Set up the bounds of the tree
	Box treeBounds;
	treeBounds.alloc(dimension);
	treeBounds.set(treeExtent, bbMid);

	// Clean up
	delete[] treeExtent;
	delete[] bbSize;
	delete[] bbMid;

	// Create the root
	if ( mRoot ) delete mRoot;
	mRoot = new KdTreeNode(0/*axis*/, NULL/*parent*/, treeBounds, NULL);
}

//===============================================================================================================================
