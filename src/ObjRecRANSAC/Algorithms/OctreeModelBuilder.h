/*
 * OctreeModelBuilder.h
 *
 *  Created on: Mar 7, 2010
 *      Author: papazov
 */

#ifndef OCTREEMODELBUILDER_H_
#define OCTREEMODELBUILDER_H_

#include "../DataStructures/ORROctree/ORROctree.h"
#include "../Algorithms/GeometryProcessor.h"
#include <vtkPolyData.h>


class OctreeModelBuilder
{
public:
	OctreeModelBuilder();
	virtual ~OctreeModelBuilder();

	enum NormalEstimationMode{PCA, MESH};

	/** The input mesh 'in' has to have normals. */
	bool buildModelOctree(vtkPolyData* mesh, double voxelsize,
			int normalEstimationNeighRadius, ORROctree* octree,
			vtkPolyData* out,
			OctreeModelBuilder::NormalEstimationMode mode = PCA);

protected:
	GeometryProcessor mGP;
};

#endif /* OCTREEMODELBUILDER_H_ */
