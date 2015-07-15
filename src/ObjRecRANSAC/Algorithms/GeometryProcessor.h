/*
 * GeometryProcessor.h
 *
 *  Created on: Mar 7, 2010
 *      Author: papazov
 */

#ifndef GEOMETRYPROCESSOR_H_
#define GEOMETRYPROCESSOR_H_

#include <BasicToolsL1/Matrix.h>
#include <BasicTools/LinearAlgebra/Vector.h>
#include <BasicTools/LinearAlgebra/Matrix.h>
#include <BasicTools/ComputationalGeometry/DataStructures/Octree/OctreeNode.h>
#include <BasicTools/ComputationalGeometry/Algorithms/PCA.h>
#include "../DataStructures/ORROctree/ORROctreeNodeData.h"
#include "../DataStructures/ORROctree/ORROctree.h"
#include <opencv/cxcore.h>

using namespace tum;

class GeometryProcessor
{
public:
	GeometryProcessor(){}
	virtual ~GeometryProcessor(){}

	/** Saves the points of the 'leafs' in 'out'. 'out' has to be a 3xN matrix, where
	  * N >= leafs.size(). */
	inline void fromLeafsToCenteredPoints(list<OctreeNode*>& leafs, double** out);

	/** 'mem' has to be a 3xN matrix, where N = (2*radius+1)^3. This is the maximal number
	  * of neighbors a leaf in 'octree' can have. 'mem' does not have to be initialized
	  * with some values, it just has to have this amount of memory. This strategy makes sense
	  * when this method is called many times - it is not needed to reserve and free memory
	  * again and again inside the method. The method saves the estimated normal in the data
	  * object of 'node'. The estimated normal will point in the same direction as 'refNormal'.
	  * The return value is the estimated normal or NULL if an estimation via PCA is not
	  * possible (if 'node' has too few neighbor).  */
	inline double* estimateOctreeNormal(double** mem, int radius,
			OctreeNode* node, ORROctree* octree, double* refNormal);

	inline void cvPolarDecomp(const double M[3][3], double* R)const;
	inline void vtkPolarDecomp(const double M[3][3], double R[3][3]) const;

protected:
	PCA mPCA;
};

//=== inline methods ====================================================================================

inline double* GeometryProcessor::estimateOctreeNormal(double** mem, int radius,
		OctreeNode* node, ORROctree* octree, double* refNormal)
{
	ORROctreeNodeData* nodeData = (ORROctreeNodeData*)node->getData();
	list<OctreeNode*> neighs;
	double eigenvals[3], eigenvecs[3][3];

	// Get the neighbors of 'node'
	octree->getFullNeighbours(node, radius, neighs);
	// Check if we have enough leafs for a PCA
	if ( neighs.size() < 3 )
	{
		nodeData->clearNormal();
		return NULL;
	}

	// Collect the points of the leafs and center them
	this->fromLeafsToCenteredPoints(neighs, mem);
	// Perform the real PCA
	mPCA.eigenComputations(mem, neighs.size(), eigenvecs, eigenvals);
	// Save the normal
	nodeData->allocNormal();
	nodeData->setNormal(eigenvecs[0][2], eigenvecs[1][2], eigenvecs[2][2]);
	double* n = nodeData->getNormal();

	// Check if we should switch the normal
	if ( Vector::dot3(n, refNormal) < 0.0 )
		Vector::mult3(n, -1.0);

	return n;
}

//=============================================================================================================

void GeometryProcessor::fromLeafsToCenteredPoints(list<OctreeNode*>& leafs, double** out)
{
	const double* p;
	double com[3] = {0.0, 0.0, 0.0};
	list<OctreeNode*>::iterator it;
	int i;

	// Compute the center of mass
	for ( it = leafs.begin() ; it != leafs.end() ; ++it )
	{
		p = ((ORROctreeNodeData*)(*it)->getData())->getPoint();
		com[0] += p[0];
		com[1] += p[1];
		com[2] += p[2];
	}
	com[0] /= (double)leafs.size();
	com[1] /= (double)leafs.size();
	com[2] /= (double)leafs.size();

	// Center the points
	for ( i = 0, it = leafs.begin() ; it != leafs.end() ; ++it, ++i )
	{
		p = ((ORROctreeNodeData*)(*it)->getData())->getPoint();
		out[0][i] = p[0] - com[0];
		out[1][i] = p[1] - com[1];
		out[2][i] = p[2] - com[2];
	}
}

//=======================================================================================================

inline void GeometryProcessor::cvPolarDecomp(const double M[3][3], double* R) const
{
	cv::Mat m(3, 3, CV_64FC1), rot(3, 3, CV_64FC1);

	m.at<double>(0, 0) = M[0][0]; m.at<double>(0, 1) = M[0][1]; m.at<double>(0, 2) = M[0][2];
	m.at<double>(1, 0) = M[1][0]; m.at<double>(1, 1) = M[1][1]; m.at<double>(1, 2) = M[1][2];
	m.at<double>(2, 0) = M[2][0]; m.at<double>(2, 1) = M[2][1]; m.at<double>(2, 2) = M[2][2];

	cv::SVD svd(m, cv::SVD::MODIFY_A);
	rot = svd.u*svd.vt;

	if ( cv::determinant(rot) < 0.0 )
	{
		svd.vt.at<double>(2,0) = -svd.vt.at<double>(2,0);
		svd.vt.at<double>(2,1) = -svd.vt.at<double>(2,1);
		svd.vt.at<double>(2,2) = -svd.vt.at<double>(2,2);
		rot = svd.u*svd.vt;
	}

	// Write back
	R[0] = rot.at<double>(0, 0); R[1] = rot.at<double>(0, 1); R[2] = rot.at<double>(0, 2);
	R[3] = rot.at<double>(1, 0); R[4] = rot.at<double>(1, 1); R[5] = rot.at<double>(1, 2);
	R[6] = rot.at<double>(2, 0); R[7] = rot.at<double>(2, 1); R[8] = rot.at<double>(2, 2);
}

//=======================================================================================================

inline void GeometryProcessor::vtkPolarDecomp(const double M[3][3], double R[3][3]) const
{
	double U[3][3], w[3], VT[3][3];
	vtkMath::SingularValueDecomposition3x3(M, U, w, VT);
	mat_mult3x3(U, VT, R);
}

//=======================================================================================================

#endif /* GEOMETRYPROCESSOR_H_ */
