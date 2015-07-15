#include "ORROctree.h"
#include "ORROctreeNodeData.h"

ORROctree::ORROctree()
{
	mFullLeafIdCounter = 0;
}

ORROctree::~ORROctree()
{
}

//===============================================================================================================================

void ORROctree::buildOctree(vtkPoints* points, double voxelsize)
{
	// Reset the id counter
	mFullLeafIdCounter = 0;

	// Call the build-method of the base class
	Octree::buildOctree(points, voxelsize);

	int i, numOfFullLeafs = mFullLeafs.size();
	// Compute the mean point for every full leaf
	for ( i = 0 ; i < numOfFullLeafs ; ++i )
		((ORROctreeNodeData*)mFullLeafs[i]->getData())->computeMeanPoint();
}

//===============================================================================================================================

void ORROctree::fillOctreeLeaf(OctreeNode* leaf, int pointId, double* point)
{
	// Get the data of the leaf
	ORROctreeNodeData* data = (ORROctreeNodeData*)leaf->getData();

	// Create a new data object if the leaf has no data
	if ( data == NULL )
	{
		data = new ORROctreeNodeData();
		leaf->setData(data);
		// Compute the 3d id of the leaf
		data->set3dId(
				(int)((leaf->getCenter()[0] - mBounds[0])/mVoxelSize),
				(int)((leaf->getCenter()[1] - mBounds[2])/mVoxelSize),
				(int)((leaf->getCenter()[2] - mBounds[4])/mVoxelSize));
		leaf->setFullLeafId(mFullLeafIdCounter++);
	}

	data->addToPoint(point);
	data->addPointId(pointId);
}

//===============================================================================================================================
