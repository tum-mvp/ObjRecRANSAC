#ifndef ORROCTREE_H_
#define ORROCTREE_H_

#include <BasicTools/ComputationalGeometry/DataStructures/Octree/Octree.h>

class ORROctree: public Octree
{
public:
	ORROctree();
	virtual ~ORROctree();

	/** Inherited from 'Octree'. */
	void fillOctreeLeaf(OctreeNode* leaf, int pointId, double* point);

	/** Inherited from 'Octree'. */
	void buildOctree(vtkPoints* points, double voxelsize);

public:
	int mFullLeafIdCounter;
};

#endif /*ORROCTREE_H_*/
