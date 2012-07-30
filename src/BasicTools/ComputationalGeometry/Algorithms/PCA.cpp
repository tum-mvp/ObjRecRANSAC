#include "PCA.h"
#include <vtkMath.h>

using namespace tum;

PCA::PCA()
{
}

PCA::~PCA()
{
}

//===============================================================================================================================================

void PCA::swap(double *v, int pos1, int pos2)
{
	double tmp = v[pos1];
	   v[pos1] = v[pos2];
	   v[pos2] = tmp;
}

//===============================================================================================================================================

void PCA::swapCol(double m[3][3], int col1, int col2)
{
	double tmp[3];
	// tmp = col1
	tmp[0] = m[0][col1];
    tmp[1] = m[1][col1];
	tmp[2] = m[2][col1];
	// col1 = col2
	m[0][col1] = m[0][col2];
	m[1][col1] = m[1][col2];
	m[2][col1] = m[2][col2];
	//  col2 = tmp
	m[0][col2] = tmp[0];
	m[1][col2] = tmp[1];
	m[2][col2] = tmp[2];	
}

//========================================================================================================================================

void PCA::doPCA(vtkPoints *points, list<int>& useIds, double eigenVectors[3][3], double eigenValues[3], double centerOfMass[3])
{
	double p[3];
	int nOfPoints = useIds.size();

	centerOfMass[0] = centerOfMass[1] = centerOfMass[2] = 0.0; 

	// Compute the mean value for the x-, y- and z-coordinate.
	for ( list<int>::iterator it = useIds.begin() ; it != useIds.end() ; ++it )
	{
		points->GetPoint(*it, p);
		centerOfMass[0] += p[0];
		centerOfMass[1] += p[1];
		centerOfMass[2] += p[2];
	}
	centerOfMass[0] /= (double)nOfPoints;
	centerOfMass[1] /= (double)nOfPoints;
	centerOfMass[2] /= (double)nOfPoints;

	// The centered points
	double **cpts = new double*[3];
	cpts[0] = new double[nOfPoints];
	cpts[1] = new double[nOfPoints];
	cpts[2] = new double[nOfPoints];

	int i = 0;
	// Subtract mean values from the true points to become centered points
	for ( list<int>::iterator it = useIds.begin() ; it != useIds.end() ; ++it, ++i )
	{
		points->GetPoint(*it, p);
		cpts[0][i] = p[0] - centerOfMass[0];
		cpts[1][i] = p[1] - centerOfMass[1];
		cpts[2][i] = p[2] - centerOfMass[2];
	}

	// Compute the eigen vectors and the eigen values
	this->eigenComputations(cpts, nOfPoints, eigenVectors, eigenValues);

	// Release memory
	delete[] cpts[0];
	delete[] cpts[1];
	delete[] cpts[2];
	delete[] cpts;
}

//========================================================================================================================================

void PCA::doPCA(vtkPoints *points, const double centerOfMass[3], vtkIdList *useIds, double eigenVectors[3][3], double eigenValues[3])
{
	double p[3];
	int i, nOfPoints = useIds->GetNumberOfIds();

	// The centered points
	double **cpts = new double*[3];
	cpts[0] = new double[nOfPoints];
	cpts[1] = new double[nOfPoints];
	cpts[2] = new double[nOfPoints];

	// Subtract the center of mass from the true points to become centered points
	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		points->GetPoint(useIds->GetId(i), p);
		cpts[0][i] = p[0] - centerOfMass[0];
		cpts[1][i] = p[1] - centerOfMass[1];
		cpts[2][i] = p[2] - centerOfMass[2];
	}

	// Compute the eigen vectors and the eigen values
	this->eigenComputations(cpts, nOfPoints, eigenVectors, eigenValues);

	// Release memory
	delete[] cpts[0];
	delete[] cpts[1];
	delete[] cpts[2];
	delete[] cpts;
}

//========================================================================================================================================

void PCA::doPCA(vtkPoints *points, vtkIdList *useIds, double eigenVectors[3][3], double eigenValues[3], double centerOfMass[3])
{
	double p[3];
	int i, nOfPoints = useIds->GetNumberOfIds();

	centerOfMass[0] = centerOfMass[1] = centerOfMass[2] = 0.0; 

	// Compute the mean value for the x-, y- and z-coordinate.
	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		points->GetPoint(useIds->GetId(i), p);
		centerOfMass[0] += p[0];
		centerOfMass[1] += p[1];
		centerOfMass[2] += p[2];
	}
	centerOfMass[0] /= (double)nOfPoints;
	centerOfMass[1] /= (double)nOfPoints;
	centerOfMass[2] /= (double)nOfPoints;

	// The centered points
	double **cpts = new double*[3];
	cpts[0] = new double[nOfPoints];
	cpts[1] = new double[nOfPoints];
	cpts[2] = new double[nOfPoints];

	// Subtract the center of mass from the true points to become centered points
	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		points->GetPoint(useIds->GetId(i), p);
		cpts[0][i] = p[0] - centerOfMass[0];
		cpts[1][i] = p[1] - centerOfMass[1];
		cpts[2][i] = p[2] - centerOfMass[2];
	}

	// Compute the eigen vectors and the eigen values
	this->eigenComputations(cpts, nOfPoints, eigenVectors, eigenValues);

	// Release memory
	delete[] cpts[0];
	delete[] cpts[1];
	delete[] cpts[2];
	delete[] cpts;
}

//========================================================================================================================================

void PCA::doPCA(vtkPoints *points, double eigenVectors[3][3], double eigenValues[3], double centerOfMass[3])
{
	double p[3];
	int i, nOfPoints = points->GetNumberOfPoints();

	centerOfMass[0] = centerOfMass[1] = centerOfMass[2] = 0.0; 

	// Compute the mean value for the x-, y- and z-coordinate.
	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		points->GetPoint(i, p);
		centerOfMass[0] += p[0];
		centerOfMass[1] += p[1];
		centerOfMass[2] += p[2];
	}
	centerOfMass[0] /= (double)nOfPoints;
	centerOfMass[1] /= (double)nOfPoints;
	centerOfMass[2] /= (double)nOfPoints;

	// The centered points
	double **cpts = new double*[3];
	cpts[0] = new double[nOfPoints];
	cpts[1] = new double[nOfPoints];
	cpts[2] = new double[nOfPoints];

	// Subtract the center of mass from the true points to become centered points
	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		points->GetPoint(i, p);
		cpts[0][i] = p[0] - centerOfMass[0];
		cpts[1][i] = p[1] - centerOfMass[1];
		cpts[2][i] = p[2] - centerOfMass[2];
	}

	// Compute the eigen vectors and the eigen values
	this->eigenComputations(cpts, nOfPoints, eigenVectors, eigenValues);

	// Release memory
	delete[] cpts[0];
	delete[] cpts[1];
	delete[] cpts[2];
	delete[] cpts;
}

//========================================================================================================================================

void PCA::doPCA(double **points, int numOfPoints, double eigenvecs[3][3], double eigenvals[3], double centerOfMass[3])
{
	int i;

	centerOfMass[0] = 0.0;
	centerOfMass[1] = 0.0;
	centerOfMass[2] = 0.0;

	// Compute the mean value for the x-, y- and z-coordinate.
	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		centerOfMass[0] += points[0][i];
		centerOfMass[1] += points[1][i];
		centerOfMass[2] += points[2][i];
	}
	centerOfMass[0] /= (double)numOfPoints;
	centerOfMass[1] /= (double)numOfPoints;
	centerOfMass[2] /= (double)numOfPoints;

	// Subtract the center of mass from the true points to become centered points
	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		points[0][i] -= centerOfMass[0];
		points[1][i] -= centerOfMass[1];
		points[2][i] -= centerOfMass[2];
	}

	// Compute the eigen vectors and the eigen values
	this->eigenComputations(points, numOfPoints, eigenvecs, eigenvals);
}

//========================================================================================================================================

void PCA::eigenComputations(double** cpts, int numOfPoints, double eigenVectors[3][3], double eigenValues[3])
{
	// Compute the covariance matrix
	double C[3][3];

	int i, j, k;
	for ( i = 0 ; i < 3 ; ++i )// matrix rows
	{
		for ( j = 0 ; j < 3 ; ++j )// matrix columns
		{
			C[i][j] = 0.0;
			for ( k = 0 ; k < numOfPoints ; ++k )				
				C[i][j] += cpts[i][k]*cpts[j][k];

			C[i][j] /= (double)numOfPoints;
		}
	}

	// Compute eigenvalues and eigenvectors
	vtkMath::Diagonalize3x3(C, eigenValues, eigenVectors);

	// Sort the eigenvalues in descending order (and the eigenvectors too)
	// First run
	if ( eigenValues[2] > eigenValues[1] )
	{
		swap(eigenValues, 2, 1);
		swapCol(eigenVectors, 2, 1);
	}
	if ( eigenValues[1] > eigenValues[0] )
	{
		swap(eigenValues, 1, 0);
		swapCol(eigenVectors, 1, 0);
	}
	// Second run
	if ( eigenValues[2] > eigenValues[1] )
	{
		swap(eigenValues, 2, 1);
		swapCol(eigenVectors, 2, 1);
	}

	// Check if the eigenvectors build a right hand basis
	if ( vtkMath::Determinant3x3(eigenVectors) < 0 )
	{
		eigenVectors[0][2] = -eigenVectors[0][2];
		eigenVectors[1][2] = -eigenVectors[1][2];
		eigenVectors[2][2] = -eigenVectors[2][2];
	}
}

//========================================================================================================================================
