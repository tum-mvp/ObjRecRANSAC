/*
 * OctreeModelBuilder.cpp
 *
 *  Created on: Mar 7, 2010
 *      Author: papazov
 */

#include "../ORRDefines.h"
#include "OctreeModelBuilder.h"
#include <BasicTools/ComputationalGeometry/DataStructures/Octree/OctreeNode.h>
#include <BasicTools/LinearAlgebra/Matrix.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>

using namespace tum;

OctreeModelBuilder::OctreeModelBuilder()
{
}

OctreeModelBuilder::~OctreeModelBuilder()
{
}

//=====================================================================================================================

bool OctreeModelBuilder::buildModelOctree(vtkPolyData* in, double voxelsize,
		int normalEstimationNeighRadius, ORROctree* octree, vtkPolyData* out,
		OctreeModelBuilder::NormalEstimationMode mode)
{
	vtkDataArray* meshNormals = in->GetPointData()->GetNormals();
	if ( !meshNormals )
	{
		fprintf(stderr, "ERROR in 'OctreeModelBuilder::%s()': input mesh has no normals.\n", __func__);
		fflush(stderr);
		return false;
	}

#if defined OBJ_REC_RANSAC_VERBOSE_1 || defined OBJ_REC_RANSAC_VERBOSE_2
	printf("OctreeModelBuilder::%s(): %i input point(s). Building model octree ...\n",
			__func__, (int)in->GetNumberOfPoints()); fflush(stdout);
#endif

	vtkPoints* outPoints = vtkPoints::New();
	  outPoints->SetDataTypeToDouble();
	vtkDoubleArray* outNormals = vtkDoubleArray::New();
	  outNormals->SetNumberOfComponents(3);

	// Build the octree
	octree->buildOctree(in->GetPoints(), voxelsize);

	// Compute the maximal number of neighbor leafs
	int pid, i = 2*normalEstimationNeighRadius + 1;
	int maxNumOfNeighbors = i*i*i, numOfFullLeafs = octree->getNumberOfFullLeafs();
	vector<OctreeNode*>& fullLeafs = octree->getFullLeafs();
	// Reserve memory for the max number of neighbors
	Matrix mem(3, maxNumOfNeighbors);
	double refNormal[3], n[3];
	ORROctreeNodeData* nodeData;

	// Estimate the normals
	for ( i = 0, pid = 0 ; i < numOfFullLeafs ; ++i )
	{
		nodeData = (ORROctreeNodeData*)fullLeafs[i]->getData();
		list<int>& ids = nodeData->getPointIds();
		refNormal[0] = refNormal[1] = refNormal[2] = 0.0;

		for ( list<int>::iterator it = ids.begin() ; it != ids.end() ; ++it )
		{
			meshNormals->GetTuple(*it, n);
			refNormal[0] += n[0];
			refNormal[1] += n[1];
			refNormal[2] += n[2];
		}
		Vector::normalize3(refNormal);

		switch ( mode )
		{
		case PCA:
			// Estimate the normal for the current node
			mGP.estimateOctreeNormal(mem.m, normalEstimationNeighRadius, fullLeafs[i], octree, refNormal);
			if ( nodeData->getNormal() )
				break; // The PCS estimation was successful -> break
			else
				;// Do NOT break -> go to the other normal estimation mode :)

		case MESH:
			nodeData->allocNormal();
			nodeData->setNormal(refNormal);
			break;
		}
		// Add the point and normal to 'out'
		if ( nodeData->getNormal() )
		{
			outPoints->InsertNextPoint(nodeData->getPoint());
			outNormals->InsertNextTuple(nodeData->getNormal());
			nodeData->setOwnPointId(pid++);
		}
	}

	// Save the points and normals
	out->SetPoints(outPoints);
	out->GetPointData()->SetNormals(outNormals);

	// Cleanup
	outPoints->Delete();
	outNormals->Delete();

#if defined OBJ_REC_RANSAC_VERBOSE_1 || defined OBJ_REC_RANSAC_VERBOSE_2
	printf("OctreeModelBuilder::%s(): %i output point(s).\n",
			__func__, (int)out->GetNumberOfPoints()); fflush(stdout);
#endif

	return true;
}

//============================================================================================================
