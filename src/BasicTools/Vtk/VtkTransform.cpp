#include "VtkTransform.h"
#include "../LinearAlgebra/Vector.h"
#include <BasicToolsL1/Matrix.h>
#include <vtkPointLocator.h>
#include <vtkPointData.h>
#include <vtkIdList.h>
#include <cmath>

using namespace tum;

VtkTransform::VtkTransform()
{
}

VtkTransform::~VtkTransform()
{
}

//===============================================================================================================================

void VtkTransform::rotateCamAboutFocalPoint(vtkCamera *cam, double rad)
{
	double x, z, pos[3], fp[3];
	cam->GetPosition(pos);
	cam->GetFocalPoint(fp);
	// Translate to the origin
	pos[0] -= fp[0];
	pos[2] -= fp[2];
	// Rotate about the origin
	x =  pos[0]*cos(rad) + pos[2]*sin(rad);
	z = -pos[0]*sin(rad) + pos[2]*cos(rad);
	pos[0] = x;
	pos[2] = z;
	// Translate back
	pos[0] += fp[0];
	pos[2] += fp[2];
	cam->SetPosition(pos);
}

//===============================================================================================================================

void VtkTransform::getBounds(vtkPoints *points, double min[3], double max[3])
{
	double p[3];
	// Init
	points->GetPoint(0, p);
	min[0] = max[0] = p[0];
	min[1] = max[1] = p[1];
	min[2] = max[2] = p[2];

	for ( int i = 1 ; i < points->GetNumberOfPoints() ; ++i )
	{
		points->GetPoint(i, p);

		if ( p[0] < min[0] )
			min[0] = p[0];
		else if ( p[0] > max[0] )
			max[0] = p[0];

		if ( p[1] < min[1] )
			min[1] = p[1];
		else if ( p[1] > max[1] )
			max[1] = p[1];

		if ( p[2] < min[2] )
			min[2] = p[2];
		else if ( p[2] > max[2] )
			max[2] = p[2];
	}
}

//===============================================================================================================================

void VtkTransform::mult(vtkPoints *pts, const double **m)
{
	int i, nOfPoints = pts->GetNumberOfPoints();
	double p1[3], p2[3];

	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		pts->GetPoint(i, p1);
		p2[0] = p1[0]*m[0][0] + p1[1]*m[0][1] + p1[2]*m[0][2];
		p2[1] = p1[0]*m[1][0] + p1[1]*m[1][1] + p1[2]*m[1][2];
		p2[2] = p1[0]*m[2][0] + p1[1]*m[2][1] + p1[2]*m[2][2];
		pts->SetPoint(i, p2);
	}
}

//===============================================================================================================================

void VtkTransform::multNormals3x3(vtkPolyData *polydata, const double **m)
{
	vtkDataArray* normals = polydata->GetPointData()->GetNormals();
	if ( !normals )
		return;

	double n[3], out[3];

	for ( int k = 0 ; k < normals->GetNumberOfTuples() ; ++k )
	{
		normals->GetTuple(k, n);
		Matrix::mult3x3(m, n, out);
		normals->SetTuple(k, out);
	}
}

//===============================================================================================================================

void VtkTransform::multNormals9(vtkPolyData *polydata, const double *T)
{
	vtkDataArray* normals = polydata->GetPointData()->GetNormals();
	if ( !normals )
		return;

	double n[3], out[3];

	for ( int k = 0 ; k < normals->GetNumberOfTuples() ; ++k )
	{
		normals->GetTuple(k, n);
		mat_mult_vec_with_mat9<double>(T, n, out);
		normals->SetTuple(k, out);
	}
}

//===============================================================================================================================

void VtkTransform::RT(vtkPoints *pts, double R[3][3], double T[3])
{
	int i, nOfPoints = pts->GetNumberOfPoints();
	double p1[3], p2[3];

	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		pts->GetPoint(i, p1);
		p2[0] = p1[0]*R[0][0] + p1[1]*R[0][1] + p1[2]*R[0][2] + T[0];
		p2[1] = p1[0]*R[1][0] + p1[1]*R[1][1] + p1[2]*R[1][2] + T[1];
		p2[2] = p1[0]*R[2][0] + p1[1]*R[2][1] + p1[2]*R[2][2] + T[2];
		pts->SetPoint(i, p2);
	}
}

//===============================================================================================================================

void VtkTransform::RT(vtkPoints *points, double **R, double T[3])
{
	int i, nOfPoints = points->GetNumberOfPoints();
	double p1[3], p2[3];

	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		points->GetPoint(i, p1);
		p2[0] = p1[0]*R[0][0] + p1[1]*R[0][1] + p1[2]*R[0][2] + T[0];
		p2[1] = p1[0]*R[1][0] + p1[1]*R[1][1] + p1[2]*R[1][2] + T[1];
		p2[2] = p1[0]*R[2][0] + p1[1]*R[2][1] + p1[2]*R[2][2] + T[2];
		points->SetPoint(i, p2);
	}
}

//===============================================================================================================================

void VtkTransform::RT(vtkPoints* src, double R[3][3], double T[3], vtkPoints* dst)
{
	int i, nOfPoints = src->GetNumberOfPoints();
	double p1[3], p2[3];

	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		// Take a source point
		src->GetPoint(i, p1);
		// Compute p2 = p1*R + T
		p2[0] = p1[0]*R[0][0] + p1[1]*R[0][1] + p1[2]*R[0][2] + T[0];
		p2[1] = p1[0]*R[1][0] + p1[1]*R[1][1] + p1[2]*R[1][2] + T[1];
		p2[2] = p1[0]*R[2][0] + p1[1]*R[2][1] + p1[2]*R[2][2] + T[2];
		// Save the results in 'dst'
		dst->InsertNextPoint(p2);
	}
}

//===============================================================================================================================

void VtkTransform::sRT(vtkPoints* src, double s, double R[3][3], double T[3], vtkPoints* dst)
{
	int i, nOfPoints = src->GetNumberOfPoints();
	double p1[3], p2[3];

	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		// Take a source point
		src->GetPoint(i, p1);
		// p2 = s*p1*R + T
		p2[0] = s*(p1[0]*R[0][0] + p1[1]*R[0][1] + p1[2]*R[0][2]) + T[0];
		p2[1] = s*(p1[0]*R[1][0] + p1[1]*R[1][1] + p1[2]*R[1][2]) + T[1];
		p2[2] = s*(p1[0]*R[2][0] + p1[1]*R[2][1] + p1[2]*R[2][2]) + T[2];
		// Save the results in 'dst'
		dst->InsertNextPoint(p2);
	}
}

//===============================================================================================================================

void VtkTransform::R(vtkPoints *pts, double R[3][3])
{
	int i, nOfPoints = pts->GetNumberOfPoints();
	double p1[3], p2[3];

	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		pts->GetPoint(i, p1);
		p2[0] = p1[0]*R[0][0] + p1[1]*R[0][1] + p1[2]*R[0][2];
		p2[1] = p1[0]*R[1][0] + p1[1]*R[1][1] + p1[2]*R[1][2];
		p2[2] = p1[0]*R[2][0] + p1[1]*R[2][1] + p1[2]*R[2][2];
		pts->SetPoint(i, p2);
	}
}

//===============================================================================================================================

void VtkTransform::S(vtkPoints *pts, double s)
{
	int i, nOfPoints = pts->GetNumberOfPoints();
	double p[3];

	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		pts->GetPoint(i, p);
		p[0] *= s;
		p[1] *= s;
		p[2] *= s;
		pts->SetPoint(i, p);
	}
}

//===============================================================================================================================

void VtkTransform::T(vtkPoints *pts, double T[3])
{
	int i, nOfPoints = pts->GetNumberOfPoints();
	double p[3];

	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		pts->GetPoint(i, p);
		p[0] += T[0];
		p[1] += T[1];
		p[2] += T[2];
		pts->SetPoint(i, p);
	}
}

//===============================================================================================================================

void VtkTransform::getRotationMatrixXYZ(double u, double v, double w, double rotmat[3][3])
{
	rotmat[0][0] = cos(v)*cos(w);
	rotmat[0][1] = sin(u)*sin(v)*cos(w) - cos(u)*sin(w);
	rotmat[0][2] = cos(u)*sin(v)*cos(w) + sin(u)*sin(w);

	rotmat[1][0] = cos(v)*sin(w);
	rotmat[1][1] = sin(u)*sin(v)*sin(w) + cos(u)*cos(w);
	rotmat[1][2] = cos(u)*sin(v)*sin(w) - sin(u)*cos(w);

	rotmat[2][0] = -sin(v);
	rotmat[2][1] =  sin(u)*cos(v);
	rotmat[2][2] =  cos(u)*cos(v);
}

//===============================================================================================================================

void VtkTransform::getRotationMatrixZYX(double u, double v, double w, double rotmat[3][3])
{
	rotmat[0][0] =  cos(v)*cos(w);
	rotmat[0][1] = -cos(v)*sin(w);
	rotmat[0][2] =  sin(v);

	rotmat[1][0] =  sin(u)*sin(v)*cos(w) + cos(u)*sin(w);
	rotmat[1][1] = -sin(u)*sin(v)*sin(w) + cos(u)*cos(w);
	rotmat[1][2] = -sin(u)*cos(v);

	rotmat[2][0] = -cos(u)*sin(v)*cos(w)+sin(u)*sin(w);
	rotmat[2][1] =  cos(u)*sin(v)*sin(w)+sin(u)*cos(w);
	rotmat[2][2] =  cos(u)*cos(v);
}

//===============================================================================================================================

void VtkTransform::getRotationMatrixX(double u, double rotmat[3][3])
{
	rotmat[0][0] = 1.0; rotmat[0][1] = 0.0;	   rotmat[0][2] =  0.0;
	rotmat[1][0] = 0.0; rotmat[1][1] = cos(u); rotmat[1][2] = -sin(u);
	rotmat[2][0] = 0.0; rotmat[2][1] = sin(u); rotmat[2][2] =  cos(u);
}

//===============================================================================================================================

void VtkTransform::getRotationMatrixY(double u, double rotmat[3][3])
{
	rotmat[0][0] =  cos(u); rotmat[0][1] = 0.0; rotmat[0][2] = sin(u);
	rotmat[1][0] =  0.0;    rotmat[1][1] = 1.0; rotmat[1][2] = 0.0;
	rotmat[2][0] = -sin(u); rotmat[2][1] = 0.0; rotmat[2][2] = cos(u);
}

//===============================================================================================================================

void VtkTransform::getRotationMatrixZ(double u, double rotmat[3][3])
{
	rotmat[0][0] = cos(u); rotmat[0][1] = -sin(u); rotmat[0][2] = 0.0;
	rotmat[1][0] = sin(u); rotmat[1][1] =  cos(u); rotmat[1][2] = 0.0;
	rotmat[2][0] = 0.0;    rotmat[2][1] =  0.0;    rotmat[2][2] = 1.0;
}

//===============================================================================================================================

void VtkTransform::getRotationMatrixX(double u, double** rotmat)
{
	rotmat[0][0] = 1.0; rotmat[0][1] = 0.0;	   rotmat[0][2] =  0.0;
	rotmat[1][0] = 0.0; rotmat[1][1] = cos(u); rotmat[1][2] = -sin(u);
	rotmat[2][0] = 0.0; rotmat[2][1] = sin(u); rotmat[2][2] =  cos(u);
}

//===============================================================================================================================

void VtkTransform::getRotationMatrixY(double u, double** rotmat)
{
	rotmat[0][0] =  cos(u); rotmat[0][1] = 0.0; rotmat[0][2] = sin(u);
	rotmat[1][0] =  0.0;    rotmat[1][1] = 1.0; rotmat[1][2] = 0.0;
	rotmat[2][0] = -sin(u); rotmat[2][1] = 0.0; rotmat[2][2] = cos(u);
}

//===============================================================================================================================

void VtkTransform::getRotationMatrixZ(double u, double** rotmat)
{
	rotmat[0][0] = cos(u); rotmat[0][1] = -sin(u); rotmat[0][2] = 0.0;
	rotmat[1][0] = sin(u); rotmat[1][1] =  cos(u); rotmat[1][2] = 0.0;
	rotmat[2][0] = 0.0;    rotmat[2][1] =  0.0;    rotmat[2][2] = 1.0;
}

//===============================================================================================================================

void VtkTransform::rotateAboutX(double u, vtkPoints *points)
{
	double mat[3][3];
	VtkTransform::getRotationMatrixX(u, mat);
	VtkTransform::R(points, mat);
}

//===============================================================================================================================

void VtkTransform::rotateAboutY(double v, vtkPoints *points)
{
	double mat[3][3];
	VtkTransform::getRotationMatrixY(v, mat);
	VtkTransform::R(points, mat);
}

//===============================================================================================================================

void VtkTransform::rotateAboutZ(double w, vtkPoints *points)
{
	double mat[3][3];
	VtkTransform::getRotationMatrixZ(w, mat);
	VtkTransform::R(points, mat);
}

//===============================================================================================================================

void VtkTransform::getCenterOfMass(vtkPoints *points, double com[3])
{
	int i, numOfPoints = points->GetNumberOfPoints();
	double p[3];
	com[0] = com[1] = com[2] = 0.0;

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		points->GetPoint(i, p);
		com[0] += p[0];
		com[1] += p[1];
		com[2] += p[2];
	}

	com[0] /= (double)numOfPoints;
	com[1] /= (double)numOfPoints;
	com[2] /= (double)numOfPoints;
}

//===============================================================================================================================

void VtkTransform::centerPoints(vtkPoints* points)
{
	double com[3];
	VtkTransform::getCenterOfMass(points, com);
	com[0] *= -1.0; com[1] *= -1.0; com[2] *= -1.0;
	VtkTransform::T(points, com);
}

//===============================================================================================================================

void VtkTransform::scaleAboutCentroid(vtkPoints* points, double s)
{
	double com[3];
	VtkTransform::getCenterOfMass(points, com);
	com[0] *= -1.0; com[1] *= -1.0; com[2] *= -1.0;
	VtkTransform::T(points, com);
	VtkTransform::S(points, s);
	com[0] *= -1.0; com[1] *= -1.0; com[2] *= -1.0;
	VtkTransform::T(points, com);
}

//===============================================================================================================================

void VtkTransform::copy(vtkPoints* src, vtkPoints* dst)
{
	int i, numOfPoints = src->GetNumberOfPoints();
	double p[3];

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		src->GetPoint(i, p);
		dst->InsertNextPoint(p);
	}
}

//===============================================================================================================================

void VtkTransform::copy(vtkPoints* src, list<int>& src_ids, vtkPoints* dst)
{
	double p[3];

	for ( list<int>::iterator it = src_ids.begin() ; it != src_ids.end() ; ++it )
	{
		src->GetPoint(*it, p);
		dst->InsertNextPoint(p);
	}
}

//===============================================================================================================================

double VtkTransform::computeMeanDist(vtkPoints* pts1, vtkPoints* pts2)
{
	int i, numOfPoints = pts1->GetNumberOfPoints();
	double p1[3], p2[3], sum = 0.0;

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		pts1->GetPoint(i, p1);
		pts2->GetPoint(i, p2);
		sum += Vector::dist3(p1, p2);
	}

	return sum/(double)numOfPoints;
}

//===============================================================================================================================

double VtkTransform::RMS(vtkPoints* pts1, vtkPoints* pts2)
{
	int i, numOfPoints = pts1->GetNumberOfPoints();
	double a[3], b[3], sumOfSquares = 0.0;

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		pts1->GetPoint(i, a);
		pts2->GetPoint(i, b);
		sumOfSquares += (pow(a[0]-b[0],2) + pow(a[1]-b[1],2) + pow(a[2]-b[2],2));
	}

	return sqrt(sumOfSquares/(double)numOfPoints);
}

//===============================================================================================================================

double VtkTransform::compMeanClosestPointDist(vtkPoints* points)
{
	vtkPolyData* poly = vtkPolyData::New();
	  poly->SetPoints(points);

	vtkPointLocator* locator = vtkPointLocator::New();
	  locator->SetDataSet(poly);
	  locator->BuildLocator();

	int i, n = points->GetNumberOfPoints();
	double p1[3], p2[3], sumDist = 0.0;
	vtkIdList* ids = vtkIdList::New();

	// Get distance between the two closest points in 'poly'.
	for ( i = 0 ; i < n ; ++i )
	{
		ids->Reset();
		points->GetPoint(i, p1);
		locator->FindClosestNPoints(2, p1, ids);

		points->GetPoint(ids->GetId(0), p1);
		points->GetPoint(ids->GetId(1), p2);

		sumDist += sqrt(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2) + pow(p1[2]-p2[2], 2));
	}

	// Clean up
	ids->Delete();
	locator->Delete();
	poly->Delete();

	return sumDist/(double)n;
}

//===============================================================================================================================
