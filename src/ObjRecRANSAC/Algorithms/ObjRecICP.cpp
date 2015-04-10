/*
 * ObjRecICP.cpp
 *
 *  Created on: Mar 23, 2011
 *      Author: papazov
 */

#include "ObjRecICP.h"

ObjRecICP::ObjRecICP()
{
	mPointLocator = NULL;
	mEpsilon = 0.00001;
}

ObjRecICP::~ObjRecICP()
{
	if ( mPointLocator ) mPointLocator->Delete();
}

//===============================================================================================================================

void ObjRecICP::doICP(ObjRecRANSAC& objrec, list<boost::shared_ptr<PointSetShape> >& detectedShapes)
{
	vtkPoints* scene_points = objrec.getInputScene();
	double p[3], mat[4][4], icp_mat[4][4], **model2scene = mat_alloc(4, 4), **scene2model = mat_alloc(4, 4);
	list<int>::iterator id;
	int i;

#ifdef OBJ_REC_RANSAC_VERBOSE
	printf("ICP refinement "); fflush(stdout);
#endif

	for ( list<boost::shared_ptr<PointSetShape> >::iterator it = detectedShapes.begin() ; it != detectedShapes.end() ; ++it )
	{
		// Get the shape
    boost::shared_ptr<PointSetShape> shape = *it;
		// This one will contain the shape points
		vtkPoints *shape_points = vtkPoints::New(VTK_DOUBLE), *out = vtkPoints::New(VTK_DOUBLE);
		  shape_points->SetNumberOfPoints((*it)->getScenePointIds().size());

		for ( i = 0, id = shape->getScenePointIds().begin() ; id != shape->getScenePointIds().end() ; ++id, ++i )
		{
			// Get the scene point
			scene_points->GetPoint(*id, p);
			// Insert the scene point to the shape point set
			shape_points->SetPoint(i, p);
		}

		// Invert the model to scene transformation
		shape->getHomogeneousRigidTransform(model2scene);
		mat_invert_rigid_transform4x4((const double**)model2scene, scene2model);

		// Use the model as target
		this->setTarget(shape->getPolyData());
		// Register the shape points to the model
		this->doRegistration(shape_points, out, mat, INT_MAX/*max iterations*/, (const double**)scene2model);

		// Invert the result such that we have again the model to scene transformation
		mat_invert_rigid_transform4x4(mat, icp_mat);
		// Save the result in the shape
		shape->setHomogeneousRigidTransform(icp_mat);

		// Cleanup
		shape_points->Delete();
		out->Delete();

#ifdef OBJ_REC_RANSAC_VERBOSE
		printf("."); fflush(stdout);
#endif
	}
#ifdef OBJ_REC_RANSAC_VERBOSE
	printf(" done\n"); fflush(stdout);
#endif

	// Cleanup
	mat_dealloc(model2scene, 4);
	mat_dealloc(scene2model, 4);
}

//===============================================================================================================================

double ObjRecICP::doRegistration(vtkPoints* source, vtkPoints* out, double mat[4][4], int maxNumOfIterations,
		const double **init_mat)
{
	int i, numOfPoints = source->GetNumberOfPoints();
	double p_tar[3], p_src[3], com_src[3], com_tar[3];
	double T[3] = {0,0,0}, R[3][3], C[3][3], Ncc[3][3], inv_numOfPoints = 1.0/(double)numOfPoints;
	double energy = DBL_MAX, energy_old;

	// Set to identity
	R[0][0] = 1; R[0][1] = 0; R[0][2] = 0;
	R[1][0] = 0; R[1][1] = 1; R[1][2] = 0;
	R[2][0] = 0; R[2][1] = 0; R[2][2] = 1;

	// Copy the points to 'out'
	out->SetNumberOfPoints(numOfPoints);
	if ( init_mat )
	{
		double p_src_transformed[3];
		for ( i = 0 ; i < numOfPoints ; ++i )
		{
			source->GetPoint(i, p_src);
			mat_mult3_by_hom_mat(init_mat, p_src, p_src_transformed);
			out->SetPoint(i, p_src_transformed);
		}
	}
	else
	{
		for ( i = 0 ; i < numOfPoints ; ++i )
		{
			source->GetPoint(i, p_src);
			out->SetPoint(i, p_src);
		}
	}

	for ( int iter = 0 ; iter < maxNumOfIterations ; ++iter )
	{
		// Initialize
		energy_old = energy;
		energy = 0.0;
		this->set3x3(C, 0.0);
		com_src[0] = com_src[1] = com_src[2] = 0.0;
		com_tar[0] = com_tar[1] = com_tar[2] = 0.0;

		for ( i = 0 ; i < numOfPoints ; ++i )
		{
			out->GetPoint(i, p_src);
			// Get the target point closest to 'p_src'
			mTarget->GetPoint(mPointLocator->FindClosestPoint(p_src), p_tar);
			// Accumulate the energy
			energy += this->sqr_dist3(p_tar, p_src);
			// Use the original point set
			source->GetPoint(i, p_src);
			// Contribute to the covariance matrix
			this->add_tensor_product_to_mat3x3(p_tar, p_src, C);
			// Contribute to both centers of mass
			this->add3(com_src, p_src);
			this->add3(com_tar, p_tar);
		}

#ifdef OBJ_REC_ICP_PRINT_ENERGY_LEVEL
		printf("energy = %lf, energy difference = %lf\n", energy, energy_old - energy); fflush(stdout);
#endif

		// If the energy increases or remains the same -> return without using the transformation from this iteration
		if ( energy_old - energy < 0.0 )
		{
			energy = energy_old;
			break;
		}

		// Compute the center of mass for the target points
		this->mult3(com_tar, inv_numOfPoints);
		this->tensor_product3x3(com_tar, com_src, Ncc);
		// Compute the covariance matrix
		this->sub3x3(C, Ncc);
		// Now compute the center of mass for the source points
		this->mult3(com_src, inv_numOfPoints);

		// Compute the optimal rotation matrix
		this->cvPolarDecomp(C, R);
		// Compute the optimal translation
		T[0] = com_tar[0] - (R[0][0]*com_src[0] + R[0][1]*com_src[1] + R[0][2]*com_src[2]);
		T[1] = com_tar[1] - (R[1][0]*com_src[0] + R[1][1]*com_src[1] + R[1][2]*com_src[2]);
		T[2] = com_tar[2] - (R[2][0]*com_src[0] + R[2][1]*com_src[1] + R[2][2]*com_src[2]);

		// Transform the source points in order to match the target points
		for ( i = 0 ; i < numOfPoints ; ++i )
		{
			source->GetPoint(i, p_src);
			this->mult3x3(R, p_src, p_tar); // Rotate the point
			this->add3(p_tar, T); // Translate the point
			// Save it
			out->SetPoint(i, p_tar);
		}

		if ( energy_old-energy < mEpsilon )
			break;
	}

	// Copy the rigid transform to 'mat'
	mat[0][0] = R[0][0]; mat[0][1] = R[0][1]; mat[0][2] = R[0][2]; mat[0][3] = T[0];
	mat[1][0] = R[1][0]; mat[1][1] = R[1][1]; mat[1][2] = R[1][2]; mat[1][3] = T[1];
	mat[2][0] = R[2][0]; mat[2][1] = R[2][1]; mat[2][2] = R[2][2]; mat[2][3] = T[2];
	mat[3][0] =          mat[3][1] =          mat[3][2] = 0.0;     mat[3][3] = 1.0;

	return sqrt(energy*inv_numOfPoints);
}

//===============================================================================================================================
