#ifndef _TUM_OCTREE_H_
#define _TUM_OCTREE_H_

#include "OctreeNode.h"
#include "../../../Stochastics/RandomGenerator.h"
#include <vtkPoints.h>
#include <cstdio>
#include <algorithm>
#include <vector>
#include <list>

using namespace std;

//#define OCTREE_TEST_MODE

namespace tum
{

class Octree
{
public:
	Octree();
	virtual ~Octree();

	void clearOctree();

	virtual void buildOctree(vtkPoints* points, double voxelsize);

	virtual void fillOctreeLeaf(OctreeNode* leaf, int pointId, double* point) = 0;

	/** This method returns a super set of the full leaves in 'input' which are intersected by the sphere with
	 * center 'p' and 'radius'. Pointers to the intersected full leafs are saved in 'out'. The method
	 * computes a super set in the sense that in general not all leafs saved in 'out' are really
	 * intersected by the sphere. The intersection test is based on the leaf radius (since its faster
	 * than checking all leaf corners and sides), so we report more leafs than we should, but still,
	 * this is a fair approximation. */
	void getLeavesIntersectedBySphere(const double* p, double radius, list<OctreeNode*>& input, list<OctreeNode*>& out);

	/** This method returns a super set of the full leafs which are intersected by the sphere with
	 * center 'p' and 'radius'. Pointers to the intersected full leafs are saved in 'out'. The method
	 * computes a super set in the sense that in general not all leafs saved in 'out' are really
	 * intersected by the sphere. The intersection test is based on the leaf radius (since its faster
	 * than checking all leaf corners and sides), so we report more leafs than we should, but still,
	 * this is a fair approximation. */
	void getFullLeafsIntersectedBySphere(const double* p, double radius, list<OctreeNode*>& out);

	/** This method returns a super set of the full leafs which are intersected or are contained in the
	 * sphere with center 'p' and 'radius'. Pointers to these full leafs are saved in 'out'.
	 * The method computes a super set in the sense that in general not all buckets saved in 'out' are
	 * really intersected by the sphere. The intersection test is based on the leaf radius (since its
	 * faster than checking all leaf corners and sides), so we report more leafs than we should, but
	 * nevertheless, this is a fair approximation. */
	void getFullLeafsWithinSphere(double* p, double radius, list<OctreeNode*>& out);

	/** This method returns a super set of the full leafs which are intersected by the plane defined by
	 * point 'p' and normal 'n'. Pointers to the intersected full leafs are saved in 'out'. The method
	 * computes a super set in the sense that in general not all leafs saved in 'out' are really
	 * intersected by the plane. The intersection test is based on the leaf radius (since its faster
	 * than checking all leaf corners and sides), so we report more leafs than we should, but
	 * nevertheless, this is a fair approximation. */
	void getFullLeafsIntersectedByPlane(const double* p, const double* n, list<OctreeNode*>& out);

	/** This method returns a super set of the full leafs which are at a distance 'dist' or closer to
	 * the plane defined by point 'p' and normal 'n'. Pointers to these full leafs are saved
	 * in 'out'. The method computes a super set in the sense that in general not all leafs saved in
	 * 'out' are really at 'dist' or closer to the plane. The proximity test is based on the leaf radius
	 * (since its faster than checking all leaf corners and sides), so we report more leafs than we should,
	 * but nevertheless, this is a fair approximation. */
	void getFullLeafsNearPlane(const double* p, const double* n, double dist, list<OctreeNode*>& out);

	/** Randomly chooses and returns a full leaf that is intersected by the sphere with center 'p' and 'radius'.
	 * Returns NULL if no leaf is intersected by the sphere. */
	OctreeNode* getRandomFullLeafOnSphere(const double* p, double radius) const;

	/** This method returns a super set containing all full leafs which are intersected by the spheres or
	 * are lying in between them. 'p' is the center and r1 < r2 are the radii of the the spheres. Pointers
	 * to the intersected full leafs are saved in 'out'.The method computes a super set in the sense that
	 * in general there are points in 'out' which do not lie in the specified range. This is due to the
	 * fact that the location tests are performed based on the distance and the radius (half of the leaf
	 * diagonal) of the tree nodes. */
	void getFullLeafsInRange(double* p, double r1, double r2, list<OctreeNode*>& out);

	/** Randomly chooses and returns a full leaf from this octree. Returns NULL if the tree does not have
	 * any full leafs. */
	OctreeNode* getRandomFullLeaf();

	/** The method pushes the pointers to the full leafs which are descendants of 'node' in 'out'. */
	inline void pushBackFullLeafsFromNode(OctreeNode* node, list<OctreeNode*>& out);

	/** Builds the vector containing the full leafs and computes a neighbourhood structure
	 * based on geometric proximity: full leafs which are geometrically next to each other will be
	 * neighbours no matter in which tree branches they are. One node can have at most 26 neighbours. */
	void buildFullLeafsVectorAndNeighbourhoodStructure();

	/** Builds the vector of full leafs. */
	void buildFullLeafsVector();

	/** Computes a neighbourhood structure based on geometric proximity: full leafs which are
	 * geometrically next to each other will be neighbours no matter in which tree branches they are.
	 * One node can have at most 26 neighbours. */
	void buildNeighbourhoodStructure();

	/** This method deletes 'node' and all its ancestors which do not contain descendants with full leafs. 'node' is
	  * also removed from the vector 'this->mFullLeafs' which contains pointers to all full leafs in the tree. */
	inline void kill(OctreeNode* node);

	/** Just calls the other kill version for every node in the list 'nodes'. Not the best performance.
	 * Should be exchanged with something better. */
//	void kill(list<OctreeNode*>& nodes);

	/** Returns the full leaf which contains (x, y, z). Returns NULL if the leaf that contains (x, y, z) is empty. */
	inline OctreeNode* getFullLeaf(double x, double y, double z);
	/** Returns the full leaf which contains 'p'. Returns NULL if the leaf that contains 'p' is empty. */
	inline OctreeNode* getFullLeaf(const double* p);

	/** Sorts the vector which contains pointers to the full leafs interpreting the pointers as integers. */
	void sortFullLeafsVector(){ sort(mFullLeafs.begin(), mFullLeafs.end());}

	/** Computes the full neighbours of 'node' and saves pointers to them in 'node'. */
	inline void collectFullNeighbours(OctreeNode* node);

	/** Computes the full neighbors of 'node' and saves pointers to them in 'neighs'. Make sure that the octree
	 * has a pre-computed neighborhood structure: call this->buildNeighbourhoodStructure() before. Note that
	 * 'node' will be part of its neighborhood. */
	void getFullNeighbours(OctreeNode* node, int neighRadius, list<OctreeNode*>& neighs);

	/** Computes the full neighbors mask of 'node' and saves it to 'mask' which has to be a 3d array of size
	  * (2*radius + 1)^3. The char[radius][radius][radius] is considered as the middle of the mask and will
	  * have the value 1, i.e., 'node' is considered as part of its own neighborhood. If char[i][j][k] == 0
	  * then the corresponding position is not occupied by a full neighbor of 'node'. */
	void getFullNeighboursMask(OctreeNode* node, int radius, char*** mask);

	vtkPoints* getInputPoints(){ return mInputPoints;}
	OctreeNode* getRoot(){ return mRoot;}
	vector<OctreeNode*>& getFullLeafs(){ return mFullLeafs;}
	int getNumberOfFullLeafs(){ return (int)mFullLeafs.size();}
	double getVoxelSize(){ return mVoxelSize;}
	const double* getBoundsOfFullLeafs(){ return mBoundsOfFullLeafs;}

	void print(FILE* stream);

#ifdef OCTREE_TEST_MODE
	int getNodeIntersectionTestCounter(){ return mNodeIntersectionTestCounter;}
	void resetNodeIntersectionTestCounter(){ mNodeIntersectionTestCounter = 0;}

	int getNodeRangeTestCounter(){ return mNodeRangeTestCounter;}
	void resetNodeRangeTestCounter(){ mNodeRangeTestCounter = 0;}
#endif

protected:
	double computeNumberOfTreeLevels(double voxelsize, double pointSetExtent);
	void fillOctree();

protected:
	OctreeNode* mRoot;
	double mVoxelSize, mWiderPointSetExtent, mBounds[6];
	int mTreeLevels;
	vtkPoints *mInputPoints;
	vector<OctreeNode*> mFullLeafs;
	RandomGenerator mRandGen;
	double mBoundsOfFullLeafs[6];

#ifdef OCTREE_TEST_MODE
	int mNodeIntersectionTestCounter, mNodeRangeTestCounter;
#endif
};

}//namespace tum

//=== inline methods ========================================================================================================

inline void tum::Octree::pushBackFullLeafsFromNode(OctreeNode* node, list<OctreeNode*>& out)
{
	list<OctreeNode*> nodes;
	nodes.push_back(node);
	int i;

	while ( nodes.size() )
	{
		 node = nodes.back(); // Get the last node
		 nodes.pop_back(); // Remove the last node

		 if ( node->hasChildren() )
			 for ( i = 0 ; i < 8 ; ++i )
				 nodes.push_back(node->getChild(i));
		 else if ( node->isFull() )
			 out.push_back(node);
	}
}

//===========================================================================================================================

inline OctreeNode* tum::Octree::getFullLeaf(const double* p)
{
	return this->getFullLeaf(p[0], p[1], p[2]);
}

//===========================================================================================================================

inline OctreeNode* tum::Octree::getFullLeaf(double x, double y, double z)
{
	// Check if p is within the octree
	if ( x <= mBounds[0] || x >= mBounds[1] ||
         y <= mBounds[2] || y >= mBounds[3] ||
         z <= mBounds[4] || z >= mBounds[5] )
		return NULL;

	int id;
	const double *center;
	OctreeNode* node = mRoot;

	// Go down to the right leaf
	while ( node->hasChildren() )
	{
		center = node->getCenter();
		id = 0;

		if ( x >= center[0] ) id |= 0x4;
		if ( y >= center[1] ) id |= 0x2;
		if ( z >= center[2] ) id |= 0x1;

		node = node->getChild(id);
	}

	if ( node->hasData() )
		return node;

	return NULL;
}

//===========================================================================================================================

inline void tum::Octree::kill(OctreeNode* node)
{
	OctreeNode *parent = node->getParent(), *child = node;
	OctreeNode** nodeNeighbours = NULL;
	int numOfNodeNeighbours = node->getNumberOfNeighbours();

	if ( numOfNodeNeighbours )
	{
		nodeNeighbours = new OctreeNode*[numOfNodeNeighbours];
		node->getNeighbours(nodeNeighbours);
	}

	if ( mFullLeafs.size() && node->isFull() )
	{
		// Remove 'node' from the vector of full leafs
		vector<OctreeNode*>::iterator node_it = lower_bound(mFullLeafs.begin(), mFullLeafs.end(), node);
		mFullLeafs.erase(node_it);
	}

	while ( parent )
	{
		// Delete the appropriate child flag of the current parent
		parent->fullDescendantsFlagsBitwiseAND( ~((unsigned char)0x01 << child->getChildNr()) );

		if ( parent->getFullDescendantsFlags() )
			// The current parent has other full descendats (not only 'child') -> break the killing process
			break;

		// Go one level up
		child = parent;
		parent = child->getParent();
	}

	// Destroy the data and all descendants of 'child'
	child->destroyData();
	child->destroyChildren();

	// Update the neighbours of node's neighbours (if there are any)
	for ( int j = 0 ; j < numOfNodeNeighbours ; ++j )
		this->collectFullNeighbours(nodeNeighbours[j]);
	// Clean up
	if ( nodeNeighbours )
		delete[] nodeNeighbours;
}

//===========================================================================================================================

inline void tum::Octree::collectFullNeighbours(OctreeNode* node)
{
	OctreeNode* neigh;
	const double s = mVoxelSize, *c = node->getCenter();
	node->allocNeighbours(26);

	// Note that 'node' is not a neighbour of itself
	neigh = this->getFullLeaf(c[0]+s, c[1]+s, c[2]+s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]  , c[1]+s, c[2]+s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]-s, c[1]+s, c[2]+s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]+s, c[1]  , c[2]+s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]  , c[1]  , c[2]+s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]-s, c[1]  , c[2]+s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]+s, c[1]-s, c[2]+s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]  , c[1]-s, c[2]+s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]-s, c[1]-s, c[2]+s); if ( neigh ) node->addNeighbour(neigh);

	neigh = this->getFullLeaf(c[0]+s, c[1]+s, c[2]  ); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]  , c[1]+s, c[2]  ); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]-s, c[1]+s, c[2]  ); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]+s, c[1]  , c[2]  ); if ( neigh ) node->addNeighbour(neigh);
//	neigh = this->getFullLeaf(c[0]  , c[1]  , c[2]  ); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]-s, c[1]  , c[2]  ); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]+s, c[1]-s, c[2]  ); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]  , c[1]-s, c[2]  ); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]-s, c[1]-s, c[2]  ); if ( neigh ) node->addNeighbour(neigh);

	neigh = this->getFullLeaf(c[0]+s, c[1]+s, c[2]-s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]  , c[1]+s, c[2]-s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]-s, c[1]+s, c[2]-s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]+s, c[1]  , c[2]-s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]  , c[1]  , c[2]-s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]-s, c[1]  , c[2]-s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]+s, c[1]-s, c[2]-s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]  , c[1]-s, c[2]-s); if ( neigh ) node->addNeighbour(neigh);
	neigh = this->getFullLeaf(c[0]-s, c[1]-s, c[2]-s); if ( neigh ) node->addNeighbour(neigh);
}

//===========================================================================================================================

#endif /*_TUM_OCTREE_H_*/
