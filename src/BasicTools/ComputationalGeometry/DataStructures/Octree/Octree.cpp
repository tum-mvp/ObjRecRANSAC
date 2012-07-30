#include "Octree.h"
#include "../../../LinearAlgebra/Vector.h"
#include "../../../Aux/NumberUtils.h"
#include "../../../Vtk/VtkTransform.h"
#include <vector>

using namespace tum;

Octree::Octree()
{
	mVoxelSize = -1.0;
	mTreeLevels = -1;
	mRoot = NULL;
	mInputPoints = NULL;
	// Set illegal bounds
	mBoundsOfFullLeafs[0] = mBounds[0] = 1.0; mBoundsOfFullLeafs[1] = mBounds[1] = -1.0;
	mBoundsOfFullLeafs[2] = mBounds[2] = 1.0; mBoundsOfFullLeafs[3] = mBounds[3] = -1.0;
	mBoundsOfFullLeafs[4] = mBounds[4] = 1.0; mBoundsOfFullLeafs[5] = mBounds[5] = -1.0;

#ifdef OCTREE_TEST_MODE
	mNodeIntersectionTestCounter = 0;
	mNodeRangeTestCounter = 0;
#endif
}

//===========================================================================================================================

Octree::~Octree()
{
	this->clearOctree();
}

//===========================================================================================================================

void Octree::clearOctree()
{
	if ( mRoot )
	{
		delete mRoot;
		mRoot = NULL;
	}
	mFullLeafs.clear();
}

//===========================================================================================================================
/*
void Octree::kill(list<OctreeNode*>& nodes)
{
	for ( list<OctreeNode*>::iterator it = nodes.begin() ; it != nodes.end() ; ++it )
		this->kill(*it);
}
*/
//===========================================================================================================================

void Octree::buildFullLeafsVectorAndNeighbourhoodStructure()
{
	if ( !mRoot )
		return;

	int i;
	OctreeNode* node;
	list<OctreeNode*> nodes;

	// Init
	nodes.push_back(mRoot);
	mFullLeafs.clear();
	mBoundsOfFullLeafs[0] = mBounds[1]; // set min x to the max x of the whole tree
	mBoundsOfFullLeafs[1] = mBounds[0]; // set max x to the min x of the whole tree
	mBoundsOfFullLeafs[2] = mBounds[3]; // ...
	mBoundsOfFullLeafs[3] = mBounds[2];
	mBoundsOfFullLeafs[4] = mBounds[5];
	mBoundsOfFullLeafs[5] = mBounds[4];

	while ( nodes.size() )
	{
		node = nodes.back();
		nodes.pop_back();

		if ( node->hasChildren() )
			for ( i = 0 ; i < 8 ; ++i )
				nodes.push_back(node->getChild(i));
		else if ( node->hasData() )
		{
			if ( node->getBounds()[0] < mBoundsOfFullLeafs[0] ) mBoundsOfFullLeafs[0] = node->getBounds()[0];
			if ( node->getBounds()[2] < mBoundsOfFullLeafs[2] ) mBoundsOfFullLeafs[2] = node->getBounds()[2];
			if ( node->getBounds()[4] < mBoundsOfFullLeafs[4] ) mBoundsOfFullLeafs[4] = node->getBounds()[4];

			if ( node->getBounds()[1] > mBoundsOfFullLeafs[1] ) mBoundsOfFullLeafs[1] = node->getBounds()[1];
			if ( node->getBounds()[3] > mBoundsOfFullLeafs[3] ) mBoundsOfFullLeafs[3] = node->getBounds()[3];
			if ( node->getBounds()[5] > mBoundsOfFullLeafs[5] ) mBoundsOfFullLeafs[5] = node->getBounds()[5];

			mFullLeafs.push_back(node);
			this->collectFullNeighbours(node);
		}
	}
	this->sortFullLeafsVector();
}

//===========================================================================================================================

void Octree::buildFullLeafsVector()
{
	if ( !mRoot )
		return;

	int i;
	OctreeNode* node;
	list<OctreeNode*> nodes;

	// Init
	nodes.push_back(mRoot);
	mFullLeafs.clear();
	mBoundsOfFullLeafs[0] = mBounds[1]; // set min x to the max x of the whole tree
	mBoundsOfFullLeafs[1] = mBounds[0]; // set max x to the min x of the whole tree
	mBoundsOfFullLeafs[2] = mBounds[3]; // ...
	mBoundsOfFullLeafs[3] = mBounds[2];
	mBoundsOfFullLeafs[4] = mBounds[5];
	mBoundsOfFullLeafs[5] = mBounds[4];

	while ( nodes.size() )
	{
		node = nodes.back();
		nodes.pop_back();

		if ( node->hasChildren() )
			for ( i = 0 ; i < 8 ; ++i )
				nodes.push_back(node->getChild(i));
		else if ( node->hasData() )
		{
			if ( node->getBounds()[0] < mBoundsOfFullLeafs[0] ) mBoundsOfFullLeafs[0] = node->getBounds()[0];
			if ( node->getBounds()[2] < mBoundsOfFullLeafs[2] ) mBoundsOfFullLeafs[2] = node->getBounds()[2];
			if ( node->getBounds()[4] < mBoundsOfFullLeafs[4] ) mBoundsOfFullLeafs[4] = node->getBounds()[4];

			if ( node->getBounds()[1] > mBoundsOfFullLeafs[1] ) mBoundsOfFullLeafs[1] = node->getBounds()[1];
			if ( node->getBounds()[3] > mBoundsOfFullLeafs[3] ) mBoundsOfFullLeafs[3] = node->getBounds()[3];
			if ( node->getBounds()[5] > mBoundsOfFullLeafs[5] ) mBoundsOfFullLeafs[5] = node->getBounds()[5];

			mFullLeafs.push_back(node);
			this->collectFullNeighbours(node);
		}
	}
	this->sortFullLeafsVector();
}

//===========================================================================================================================

void Octree::buildNeighbourhoodStructure()
{
	if ( !mRoot )
		return;

	int i;
	OctreeNode* node;
	list<OctreeNode*> nodes;
	nodes.push_back(mRoot);

	while ( nodes.size() )
	{
		node = nodes.back();
		nodes.pop_back();

		if ( node->hasChildren() )
			for ( i = 0 ; i < 8 ; ++i )
				nodes.push_back(node->getChild(i));
		else if ( node->hasData() )
			this->collectFullNeighbours(node);
	}
}

//===========================================================================================================================

void Octree::buildOctree(vtkPoints* points, double voxelsize)
{
	if ( voxelsize <= 0 )
		return;

	this->clearOctree();

	mInputPoints = points;
	mVoxelSize = voxelsize;

	double min[3], max[3];
	VtkTransform::getBounds(points, min, max);

	// Get the extent of the input point set
	int id = 0;
	double tmp;
	mWiderPointSetExtent = max[0] - min[0];
	tmp = max[1] - min[1];
	if ( tmp > mWiderPointSetExtent ) {mWiderPointSetExtent = tmp; id = 1;}
	tmp = max[2] - min[2];
	if ( tmp > mWiderPointSetExtent ) {mWiderPointSetExtent = tmp; id = 2;}

	// Enlarge the extent a little bit in order to avoid points lying exact
	// on the boundaries of the octree
	mWiderPointSetExtent += (max[id]-min[id])*0.00001;

	// Compute the center of the root
	double center[3] = {(min[0]+max[0])/2.0, (min[1]+max[1])/2.0, (min[2]+max[2])/2.0};
	// Compute the number of octree levels and the bounds of the root
	double rootExtent = this->computeNumberOfTreeLevels(mVoxelSize, mWiderPointSetExtent);

	double eHalf = rootExtent/2.0;
	mBounds[0] = center[0] - eHalf;
	mBounds[1] = center[0] + eHalf;
	mBounds[2] = center[1] - eHalf;
	mBounds[3] = center[1] + eHalf;
	mBounds[4] = center[2] - eHalf;
	mBounds[5] = center[2] + eHalf;

	// Create and init the root
	mRoot = new OctreeNode();
	  mRoot->setCenter(center);
	  mRoot->setBounds(mBounds);
	  mRoot->setParent(NULL);

	// Fill the leafs with some stuff
	this->fillOctree();
	// Save pointers to all full leafs in a list and build a neighbourhood structure
	this->buildFullLeafsVectorAndNeighbourhoodStructure();
}

//===========================================================================================================================

void Octree::fillOctree()
{
	double p[3];
	const double *center;
	int i, l, id, numOfPoints = mInputPoints->GetNumberOfPoints();
	OctreeNode* node;

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		// Some initialization
		mInputPoints->GetPoint(i, p);
		node = mRoot;

		// Go down to the right leaf
		for ( l = 0 ; l < mTreeLevels ; ++l )
		{
			node->createChildren(); // If this node already has children -> nothing will happen
			center = node->getCenter();
			id = 0;

			if ( p[0] >= center[0] ) id |= 4;
			if ( p[1] >= center[1] ) id |= 2;
			if ( p[2] >= center[2] ) id |= 1;

			node->fullDescendantsFlagsBitwiseOR((unsigned char)0x01 << id);
			node = node->getChild(id);
		}

		this->fillOctreeLeaf(node, i, p);
	}
}

//===========================================================================================================================

double Octree::computeNumberOfTreeLevels(double voxelsize, double pointSetExtent)
{
	double treeExtent = voxelsize;
	mTreeLevels = 0;

	while ( treeExtent < pointSetExtent )
	{
		treeExtent *= 2.0;
		++mTreeLevels;
	}

	return treeExtent;
}

//===========================================================================================================================

void Octree::getLeavesIntersectedBySphere(const double* p, double radius, list<OctreeNode*>& input, list<OctreeNode*>& out)
{
	for ( list<OctreeNode*>::iterator it = input.begin() ; it != input.end() ; ++it )
		// Check if the sphere intersects the current node
		if ( fabs(radius - Vector::dist3(p, (*it)->getCenter())) <= (*it)->getVoxelRadius() )
			out.push_back(*it);
}

//===========================================================================================================================

void Octree::getFullLeafsIntersectedBySphere(const double* p, double radius, list<OctreeNode*>& out)
{
	list<OctreeNode*> nodes;
	nodes.push_back(mRoot);
	int i;

	while ( nodes.size() )
	{
		// Get the last element in the list
		OctreeNode* node = nodes.back();
		// Remove the last element from the list
		nodes.pop_back();

		// Check if the sphere intersects the current node
		if ( fabs(radius - Vector::dist3(p, node->getCenter())) <= node->getVoxelRadius() )
		{
			// We have an intersection -> push back the children of the current node
			if ( node->hasChildren() )
			{
				for ( i = 0 ; i < 8 ; ++i )
					nodes.push_back(node->getChild(i));
			}
			else if ( node->isFull() )
				out.push_back(node);
		}
	}
}

//===========================================================================================================================

void Octree::getFullLeafsWithinSphere(double* p, double radius, list<OctreeNode*>& out)
{
	list<OctreeNode*> nodes;
	nodes.push_back(mRoot);
	int i;
	double nodeCenterToSphereCenterDist;

	while ( nodes.size() )
	{
		// Get the last element in the list
		OctreeNode* node = nodes.back();
		// Remove the last element from the list
		nodes.pop_back();

		// Get the distance between the node center and the sphere center
		nodeCenterToSphereCenterDist = Vector::dist3(p, node->getCenter());

		// Check if the current node is completely contained in the sphere.
		// If yes -> add all its full children to 'out'.
		if ( nodeCenterToSphereCenterDist + node->getVoxelRadius() <= radius )
			this->pushBackFullLeafsFromNode(node, out);
		// Check if the sphere intersects the current node
		else if ( fabs(radius - nodeCenterToSphereCenterDist) <= node->getVoxelRadius() )
		{
			// We have an intersection -> push back the children of the current node
			if ( node->hasChildren() )
				for ( i = 0 ; i < 8 ; ++i )
					nodes.push_back(node->getChild(i));
			else if ( node->isFull() )
				out.push_back(node);
		}
	}
}

//===========================================================================================================================

void Octree::getFullLeafsIntersectedByPlane(const double* p, const double* n, list<OctreeNode*>& out)
{
	list<OctreeNode*> nodes;
	nodes.push_back(mRoot);
	int i;

	while ( nodes.size() )
	{
		// Get the last element in the list
		OctreeNode* node = nodes.back();
		// Remove the last element in the list
		nodes.pop_back();

		// Check if the plane intersects the current node
		if ( fabs(Vector::signedDistToPlane3(p, n, node->getCenter())) <= node->getVoxelRadius() )
		{
			// We have an intersection -> push back the children of the current node
			if ( node->hasChildren() )
			{
				for ( i = 0 ; i < 8 ; ++i )
					nodes.push_back(node->getChild(i));
			}
			else if ( node->isFull() )
				out.push_back(node);
		}
	}
}

//===========================================================================================================================

void Octree::getFullLeafsNearPlane(const double* p, const double* n, double dist, list<OctreeNode*>& out)
{
	list<OctreeNode*> nodes;
	nodes.push_back(mRoot);
	int i;

	while ( nodes.size() )
	{
		// Get the last element in the list
		OctreeNode* node = nodes.back();
		// Remove the last element from the list
		nodes.pop_back();

		// Check if the sphere intersects the current node
		if ( fabs(Vector::signedDistToPlane3(p, n, node->getCenter())) <= dist + node->getVoxelRadius() )
		{
			// We have an intersection -> push back the children of the current node
			if ( node->hasChildren() )
			{
				for ( i = 0 ; i < 8 ; ++i )
					nodes.push_back(node->getChild(i));
			}
			else if ( node->isFull() )
				out.push_back(node);
		}
	}
}

//===========================================================================================================================

OctreeNode* Octree::getRandomFullLeafOnSphere(const double* p, double radius) const
{
	list<OctreeNode*> nodes;
	nodes.push_back(mRoot);
	int i, rand_id;
	vector<int> tmp_ids;
	tmp_ids.reserve(8);

	while ( nodes.size() )
	{
		// Get the last element in the list
		OctreeNode* node = nodes.back();
		// Remove the last element from the list
		nodes.pop_back();

		// Check if the sphere intersects the current node
		if ( fabs(radius - Vector::dist3(p, node->getCenter())) <= node->getVoxelRadius() )
		{
			// We have an intersection -> push back the children of the current node
			if ( node->hasChildren() )
			{
				// Prepare the tmp id vector
				for ( i = 0 ; i < 8 ; ++i )
					tmp_ids.push_back(i);

				// Push back the children in random order
				for ( i = 0 ; i < 8 ; ++i )
				{
					rand_id = mRandGen.getRandomInteger(0, tmp_ids.size()-1);
					nodes.push_back(node->getChild(tmp_ids[rand_id]));
					// Remove the randomly selected id
					tmp_ids.erase(tmp_ids.begin() + rand_id);
				}
			}
			else if ( node->isFull() )
				return node;
		}
	}

	return NULL;
}

//===========================================================================================================================

void Octree::getFullLeafsInRange(double* p, double r1, double r2, list<OctreeNode*>& out)
{
	list<OctreeNode*> nodes;
	nodes.push_back(mRoot);
	int i;
	double d1, d2, dist;

	while ( nodes.size() )
	{
		// Get the last element in the list
		OctreeNode* node = nodes.back();
		// Remove the last element in the list
		nodes.pop_back();

#ifdef OCTREE_TEST_MODE
		++mNodeRangeTestCounter;
#endif

		// Get the distance between the sphere radius and the node center
		dist = Vector::dist3(p, node->getCenter());
		d1 = dist + node->getVoxelRadius();
		d2 = dist - node->getVoxelRadius();

		if ( d1 >= r1 && d2 <= r2 )
		{
			// Check if we have the special case that the voxel is completely between both spheres
			if ( d1 <= r2 && d2 >= r1 )
				this->pushBackFullLeafsFromNode(node, out);
			else
			{
				if ( node->hasChildren() )
				{
					// Push back the children for further processing
					for ( i = 0 ; i < 8 ; ++i )
						nodes.push_back(node->getChild(i));
				}
				// We have reached a leaf -> save it in 'out' if it's full
				else if ( node->isFull() )
					out.push_back(node);
			}
		}
	}
}

//===========================================================================================================================

OctreeNode* Octree::getRandomFullLeaf()
{
	list<OctreeNode*> nodes;
	nodes.push_back(mRoot);
	int i, rand_id;
	vector<int> tmp_ids;
	tmp_ids.reserve(8);

	while ( nodes.size() )
	{
		// Get the last element in the list
		OctreeNode* node = nodes.back();
		// Remove the last element from the list
		nodes.pop_back();

		// Push back the children of the current node
		if ( node->hasChildren() )
		{
			// Prepare the tmp id vector
			for ( i = 0 ; i < 8 ; ++i )
				tmp_ids.push_back(i);

			// Push back the children in random order
			for ( i = 0 ; i < 8 ; ++i )
			{
				rand_id = mRandGen.getRandomInteger(0, tmp_ids.size()-1);
				nodes.push_back(node->getChild(tmp_ids[rand_id]));
				// Remove the randomly selected id
				tmp_ids.erase(tmp_ids.begin() + rand_id);
			}
		}
		else if ( node->isFull() )
			return node;
	}

	return NULL;
}

//===========================================================================================================================

void Octree::getFullNeighbours(OctreeNode* node, int neighRadius, list<OctreeNode*>& neighs)
{
	int i; double maxdist = (double)neighRadius*mVoxelSize + mVoxelSize*0.001;
	list<OctreeNode*> todo1, todo2, toReset;
	list<OctreeNode*> *ptr1 = &todo1, *ptr2 = &todo2, *ptrTmp;
	OctreeNode **tmp_neighs, *tmp_node;
	const double *node_center = node->getCenter();
	// Init
	node->setNeighCheckToTrue();
	toReset.push_back(node);
	todo1.push_back(node);

	while ( ptr1->size() )
	{
		while ( ptr1->size() )
		{
			tmp_node = ptr1->back();
			ptr1->pop_back();
			tmp_neighs = tmp_node->getNeighbours();

			for ( i = 0 ; i < tmp_node->getNumberOfNeighbours() ; ++i )
			{
				if ( tmp_neighs[i]->getNeighCheck() ) continue;
				// Mark that node as checked
				tmp_neighs[i]->setNeighCheckToTrue();
				// The flag of this node has to be reset at the end of this method
				toReset.push_back(tmp_neighs[i]);
				// Check the distance
				if ( Vector::dist3(tmp_neighs[i]->getCenter(), node_center) >= maxdist )
					continue;
				ptr2->push_back(tmp_neighs[i]);
				neighs.push_back(tmp_neighs[i]);
			}
		}
		// Swap the pointers
		ptrTmp = ptr1;
		ptr1 = ptr2;
		ptr2 = ptrTmp;
	}
	neighs.push_back(node);

	// Set all node flags again to zero.
	for ( list<OctreeNode*>::iterator it = toReset.begin() ; it != toReset.end() ; ++it )
		(*it)->setNeighCheckToFalse();
}

//===========================================================================================================================

void Octree::getFullNeighboursMask(OctreeNode* node, int radius, char*** mask)
{
	int i, x, y, z; double maxdist = (double)radius*mVoxelSize + mVoxelSize*0.001;
	list<OctreeNode*> todo1, todo2, toReset;
	list<OctreeNode*> *ptr1 = &todo1, *ptr2 = &todo2, *ptrTmp;
	OctreeNode **tmp_neighs, *tmp_node;
	const double *node_center = node->getCenter();
	// Init
	node->setNeighCheckToTrue();
	toReset.push_back(node);
	todo1.push_back(node);

	// 'node' is part of its own neighborhood
	mask[radius][radius][radius] = 1;

	while ( ptr1->size() )
	{
		while ( ptr1->size() )
		{
			tmp_node = ptr1->back();
			ptr1->pop_back();
			tmp_neighs = tmp_node->getNeighbours();

			for ( i = 0 ; i < tmp_node->getNumberOfNeighbours() ; ++i )
			{
				if ( tmp_neighs[i]->getNeighCheck() ) continue;
				// Mark that node as checked
				tmp_neighs[i]->setNeighCheckToTrue();
				// The flag of this node has to be reset at the end of this method
				toReset.push_back(tmp_neighs[i]);
				// Check the distance
				if ( Vector::dist3(tmp_neighs[i]->getCenter(), node_center) >= maxdist )
					continue;
				ptr2->push_back(tmp_neighs[i]);
				// Compute the id of the neighbor in the 'mask'
				x = NumberUtils::roundDouble((tmp_neighs[i]->getCenter()[0] - node_center[0])/mVoxelSize) + radius;
				y = NumberUtils::roundDouble((tmp_neighs[i]->getCenter()[1] - node_center[1])/mVoxelSize) + radius;
				z = NumberUtils::roundDouble((tmp_neighs[i]->getCenter()[2] - node_center[2])/mVoxelSize) + radius;
				mask[x][y][z] = 1;
			}
		}
		// Swap the pointers
		ptrTmp = ptr1;
		ptr1 = ptr2;
		ptr2 = ptrTmp;
	}

	// Set all node flags again to zero.
	for ( list<OctreeNode*>::iterator it = toReset.begin() ; it != toReset.end() ; ++it )
		(*it)->setNeighCheckToFalse();
}

//===========================================================================================================================

void Octree::print(FILE* stream)
{
	fprintf(stream, "-------------------------------------- Octree info --------------------------------------\n\n");
	if ( mRoot )
	{
		double *bounds = mRoot->getBounds();
		fprintf(stream, "Number of input points = %i\nEnlarged input point set extent = %lf\nOctree extent = %lf\n",
				(int)mInputPoints->GetNumberOfPoints(), mWiderPointSetExtent, mRoot->getNodeSize());
		fprintf(stream, "Octree bounds = [%lf, %lf] x [%lf, %lf] x [%lf, %lf]\n"
				"Voxel size = %lf\n", bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5], mVoxelSize);
		fprintf(stream, "Full leafs center bounds: [%lf, %lf] x [%lf, %lf] x [%lf, %lf]\n",
				mBoundsOfFullLeafs[0], mBoundsOfFullLeafs[1], mBoundsOfFullLeafs[2],
				mBoundsOfFullLeafs[3], mBoundsOfFullLeafs[4], mBoundsOfFullLeafs[5]);
		fprintf(stream, "%i full leafs.\n", (int)mFullLeafs.size());
		fprintf(stream, "%i tree levels.\n", mTreeLevels);
	}
	else
		fprintf(stream, "The octree is not build.\n");
	fprintf(stream, "\n-----------------------------------------------------------------------------------------\n");
	fflush(stream);
}

//===========================================================================================================================
