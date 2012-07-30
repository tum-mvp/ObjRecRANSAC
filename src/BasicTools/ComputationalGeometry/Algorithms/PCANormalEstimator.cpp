#include "PCANormalEstimator.h"
#include "PCA.h"
#include "../../LinearAlgebra/Vector.h"
#include <vtkPointLocator.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkIdList.h>

using namespace tum;

PCANormalEstimator::PCANormalEstimator()
{
}

PCANormalEstimator::~PCANormalEstimator()
{
}

//===============================================================================================================================

void PCANormalEstimator::estimateNormals(vtkPolyData* data, double neighRadius)
{
	vtkPointLocator* locator = vtkPointLocator::New();
	  locator->SetDataSet(data);
	  locator->BuildLocator();

	int i, numOfPoints = data->GetNumberOfPoints();
	vtkDoubleArray* normals = vtkDoubleArray::New();
	  normals->SetNumberOfComponents(3);
	  normals->SetNumberOfTuples(numOfPoints);

	PCA pca;
	double p[3], com[3], eigenvals[3], eigenvecs[3][3];
	vtkPoints* points = data->GetPoints();
	vtkIdList* neighs = vtkIdList::New();

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		points->GetPoint(i, p);
		neighs->Reset();
		locator->FindPointsWithinRadius(neighRadius, p, neighs);
		if ( neighs->GetNumberOfIds() < 3 )
		{
			normals->SetTuple3(i, 0.0, 0.0, 0.0);
			continue;
		}
		// Perform PCA
		pca.doPCA(points, neighs, eigenvecs, eigenvals, com);
		// Save the normal
		normals->SetTuple3(i, eigenvecs[0][2], eigenvecs[1][2], eigenvecs[2][2]);
	}

	data->GetPointData()->SetNormals(normals);

	// Clean up
	locator->Delete();
	normals->Delete();
	neighs->Delete();
}

//===============================================================================================================================

void PCANormalEstimator::orientNormalsForStarShape(vtkPolyData* data, double* starPoint)
{
	vtkDataArray* normals = data->GetPointData()->GetNormals();

	if ( !normals )
	{
		fprintf(stderr, "ERROR in 'PCANormalEstimator::%s()': no normals.\n", __func__); fflush(stderr);
		return;
	}

	int i, numOfPoints = data->GetNumberOfPoints();
	double *n, p[3], line[3];

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		data->GetPoint(i, p);
		n = normals->GetTuple3(i);
		Vector::diff(p, starPoint, line);
		Vector::normalize3(line);
		if ( Vector::dot3(line, n) < 0 )
		{
			n[0] *= -1.0;
			n[1] *= -1.0;
			n[2] *= -1.0;
			normals->SetTuple3(i, n[0], n[1], n[2]);
		}
	}
}

//===============================================================================================================================
