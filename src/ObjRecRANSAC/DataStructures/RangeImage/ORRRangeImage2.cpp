/*
 * ORRRangeImage2.cpp
 *
 *  Created on: Jun 4, 2010
 *      Author: papazov
 */

#include "ORRRangeImage2.h"
#include <BasicToolsL1/Vector.h>
#include "../ORROctree/ORROctreeNodeData.h"

ORRRangeImage2::ORRRangeImage2()
{
	mPixels = NULL;
	mNormals = NULL;
	mGridSets = NULL;
	mShapesGrid = NULL;
	mOctreeNodes = NULL;
	mInvPixelSize = 0.0;
}

ORRRangeImage2::~ORRRangeImage2()
{
	this->clear();
}

//============================================================================================

void ORRRangeImage2::clearGridSets()
{
	int i, j;
	if ( mGridSets )
	{
		for ( i = 0 ; i < mNumOfPixelsX ; ++i )
		{
			for ( j = 0 ; j < mNumOfPixelsY ; ++j )
				if ( mGridSets[i][j] )
					delete mGridSets[i][j];
			delete[] mGridSets[i];
		}
		delete[] mGridSets;
		mGridSets = NULL;
	}
	mFullSets.clear();
}

//============================================================================================

void ORRRangeImage2::clear()
{
	int i, j;

	this->clearGridSets();

	if ( mPixels )
	{
		for ( i = 0 ; i < mNumOfPixelsX ; ++i )
		{
			for ( j = 0 ; j < mNumOfPixelsY ; ++j )
				if ( mPixels[i][j] )
					delete mPixels[i][j];
			delete[] mPixels[i];
		}
		delete[] mPixels;
		mPixels = NULL;
	}

	if ( mNormals )
	{
		for ( i = 0 ; i < mNumOfPixelsX ; ++i )
		{
			for ( j = 0 ; j < mNumOfPixelsY ; ++j )
				if ( mNormals[i][j] )
					delete mNormals[i][j];
			delete[] mNormals[i];
		}
		delete[] mNormals;
		mNormals = NULL;
	}

	if ( mShapesGrid )
	{
		for ( i = 0 ; i < mNumOfPixelsX ; ++i )
		{
			for ( j = 0 ; j < mNumOfPixelsY ; ++j )
				if ( mShapesGrid[i][j] )
					delete mShapesGrid[i][j];
			delete[] mShapesGrid[i];
		}
		delete[] mShapesGrid;
		mShapesGrid = NULL;
	}

	if ( mOctreeNodes )
	{
		for ( i = 0 ; i < mNumOfPixelsX ; ++i )
		{
			for ( j = 0 ; j < mNumOfPixelsY ; ++j )
				if ( mOctreeNodes[i][j] )
					delete mOctreeNodes[i][j];
			delete[] mOctreeNodes[i];
		}
		delete[] mOctreeNodes;
		mOctreeNodes = NULL;
	}

	mNumOfPixelsX = mNumOfPixelsY = mNumOfPixels = 0;
	mExtentX = mExtentY = mPixelSize = mInvPixelSize = 0.0;
}

//=================================================================================================================

void ORRRangeImage2::projectOctree(ORROctree* octree)
{
	this->clear();

	// Make some initializations
	mPixelSize = octree->getVoxelSize();
	mInvPixelSize = 1.0/mPixelSize;
	const double* fullLeafsBounds = octree->getBoundsOfFullLeafs();

	mBounds[0] = fullLeafsBounds[0]; mBounds[1] = fullLeafsBounds[1];
	mBounds[2] = fullLeafsBounds[2]; mBounds[3] = fullLeafsBounds[3];

	mExtentX = fullLeafsBounds[1]-fullLeafsBounds[0];
	mExtentY = fullLeafsBounds[3]-fullLeafsBounds[2];

	mNumOfPixelsX = (int)(mExtentX/mPixelSize + 0.5);
	mNumOfPixelsY = (int)(mExtentY/mPixelSize + 0.5);
	mNumOfPixels = mNumOfPixelsX*mNumOfPixelsY;

	int i, j;
	// Alloc and init memory for the pixels
	mPixels = new double_2**[mNumOfPixelsX];
	for ( i = 0 ; i < mNumOfPixelsX ; ++i )
	{
		mPixels[i] = new double_2*[mNumOfPixelsY];
		for ( j = 0 ; j < mNumOfPixelsY ; ++j )
			mPixels[i][j] = NULL;
	}
	// Alloc and init memory for the normals
	mNormals = new double_3**[mNumOfPixelsX];
	for ( i = 0 ; i < mNumOfPixelsX ; ++i )
	{
		mNormals[i] = new double_3*[mNumOfPixelsY];
		for ( j = 0 ; j < mNumOfPixelsY ; ++j )
			mNormals[i][j] = NULL;
	}

	// Alloc and init memory for the shapes grid
	mShapesGrid = new list<int>**[mNumOfPixelsX];
	for ( i = 0 ; i < mNumOfPixelsX ; ++i )
	{
		mShapesGrid[i] = new list<int>*[mNumOfPixelsY];
		for ( j = 0 ; j < mNumOfPixelsY ; ++j )
			mShapesGrid[i][j] = NULL;
	}
	// Alloc memory for the sets
	mGridSets = new ORRRangeImage2PixelSet**[mNumOfPixelsX];
	for ( i = 0 ; i < mNumOfPixelsX ; ++i )
	{
		mGridSets[i] = new ORRRangeImage2PixelSet*[mNumOfPixelsY];
		for ( j = 0 ; j < mNumOfPixelsY ; ++j )
			mGridSets[i][j] = NULL;
	}
	// Alloc memory for the octree nodes
	mOctreeNodes = new list<OctreeNode*>**[mNumOfPixelsX];
	for ( i = 0 ; i < mNumOfPixelsX ; ++i )
	{
		mOctreeNodes[i] = new list<OctreeNode*>*[mNumOfPixelsY];
		for ( j = 0 ; j < mNumOfPixelsY ; ++j )
			mOctreeNodes[i][j] = NULL;
	}

	// Project the octree full leaves onto the xy-plane
	vector<OctreeNode*>& fullLeafs = octree->getFullLeafs();
	int x, y, numOfFullLeafs = fullLeafs.size();

	for ( i = 0 ; i < numOfFullLeafs ; ++i )
	{
		this->getPixelCoordinates(fullLeafs[i]->getCenter(), x, y);
		// If there is no set at this position -> create one
		if ( mGridSets[x][y] == NULL )
		{
			mGridSets[x][y] = new ORRRangeImage2PixelSet(x, y);
			mFullSets.push_back(mGridSets[x][y]);
		}

		// Insert the full octree leaf at the right position in the set
		mGridSets[x][y]->insert(fullLeafs[i]);
	}
}

//=================================================================================================================

void ORRRangeImage2::buildFromOctree(ORROctree* octree, double front_eps, double back_eps)
{
	this->projectOctree(octree);

	double best_front, front, best_back, back, p[3], n[3];
	int x, y, id_z1, id_z2, maxlen, len;
	bool has_normal;

	for ( list<ORRRangeImage2PixelSet*>::iterator gridSet = mFullSets.begin() ; gridSet != mFullSets.end() ; ++gridSet )
	{
		// Get the first node in the set
		set<OctreeNode*>::iterator node = (*gridSet)->begin();
		// Init run
		best_front = front = (*node)->getBounds()[4];
		best_back = back = (*node)->getBounds()[5];
		id_z1 = ((ORROctreeNodeData*)(*node)->getData())->get3dId()[2];
		maxlen = len = 1;

		// Find the longest 1D "connected component" at the current position
		for ( ++node ; node != (*gridSet)->end() ; ++node, id_z1 = id_z2 )
		{
			id_z2 = ((ORROctreeNodeData*)(*node)->getData())->get3dId()[2];
			back = (*node)->getBounds()[5];

			if ( abs(id_z1 - id_z2) > 1 ) // This connected component is over
			{
				// Start a new connected component
				front = (*node)->getBounds()[4];
				len = 1;
			}
			else // This connected component is still ongoing
			{
				++len;
				if ( len > maxlen )
				{
					// This connected component is the longest one
					maxlen = len;
					best_back = back;
					best_front = front;
				}
			}
		}

		x = (*gridSet)->x();
		y = (*gridSet)->y();

		mPixels[x][y] = new double_2();
		mPixels[x][y]->x = best_front - front_eps;
		mPixels[x][y]->y = best_back + back_eps;

		if ( mOctreeNodes[x][y] )
		{
			fprintf(stderr, "WARNING in 'ORRRangeImage2::%s()': there should be no octree nodes at [%i, %i]\n", __func__, x, y);
			delete mOctreeNodes[x][y];
		}
		mOctreeNodes[x][y] = new list<OctreeNode*>();

		// Save the nodes belonging to the longest 1D component
		vec_set3(p, 0.0);
		vec_set3(n, 0.0);
		has_normal = false;
		for ( set<OctreeNode*>::iterator it = (*gridSet)->begin() ; it != (*gridSet)->end() ; ++it )
		{
			if ( best_front < (*it)->getCenter()[2] && (*it)->getCenter()[2] < best_back )
			{
				mOctreeNodes[x][y]->push_back(*it);
				// Get the node data
				ORROctreeNodeData* node_data = ((ORROctreeNodeData*)(*it)->getData());
				// Accumulate to 'p'
				vec_add3(p, node_data->getPoint());
				// Do we have a normal?
				if ( node_data->getNormal() )
				{
					vec_add3(n, node_data->getNormal());
					has_normal = true;
				}
			}
		}
		if ( mOctreeNodes[x][y]->size() <= 0 )
			fprintf(stderr, "WARNING in 'ORRRangeImage2::%s()': there has to be at least one node at [%i, %i].\n", __func__, x, y);
		else
		{
			vec_mult3(p, 1.0/(double)mOctreeNodes[x][y]->size());
			if ( has_normal )
			{
				vec_normalize3(n);
				if ( mNormals[x][y] )
				{
					fprintf(stderr, "WARNING in 'ORRRangeImage2::%s()': there should be no normal at [%i, %i].\n", __func__, x, y);
					delete mNormals[x][y];
				}
				mNormals[x][y] = new double_3(n);
			}
		}

		// Save the point
		(*gridSet)->set_p(p);
	}
}

//=================================================================================================================
