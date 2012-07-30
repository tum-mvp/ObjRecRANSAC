#include "OptimalTransformation.h"
#include "../../LinearAlgebra/Vector.h"
#include "../../LinearAlgebra/AnalyticGeometry.h"
#include <vtkMath.h>

using namespace tum;


OptimalTransformation::OptimalTransformation()
{
}

OptimalTransformation::~OptimalTransformation()
{
}

//==========================================================================================================================

void OptimalTransformation::getOptimalRotation(const double** points1, const double** points2, int numOfPoints, double** rotmat) const
{
	// Estimate rotation
	double H[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};
	const double *p1, *p2;

	for ( int i = 0 ; i < numOfPoints ; ++i )
	{
		p1 = points1[i];
		p2 = points2[i];
		H[0][0] += p1[0]*p2[0]; H[0][1] += p1[0]*p2[1]; H[0][2] += p1[0]*p2[2];
		H[1][0] += p1[1]*p2[0]; H[1][1] += p1[1]*p2[1]; H[1][2] += p1[1]*p2[2];
		H[2][0] += p1[2]*p2[0]; H[2][1] += p1[2]*p2[1]; H[2][2] += p1[2]*p2[2];
	}

	double U[3][3], UT[3][3], w[3], V[3][3], VT[3][3], tmp[3][3];
	vtkMath::SingularValueDecomposition3x3(H, U, w, VT);
	vtkMath::Transpose3x3(U, UT);
	vtkMath::Transpose3x3(VT, V);
	// Compute the optimal rotation matrix
	vtkMath::Multiply3x3(V, UT, tmp);
	rotmat[0][0] = tmp[0][0]; rotmat[0][1] = tmp[0][1]; rotmat[0][2] = tmp[0][2];
	rotmat[1][0] = tmp[1][0]; rotmat[1][1] = tmp[1][1]; rotmat[1][2] = tmp[1][2];
	rotmat[2][0] = tmp[2][0]; rotmat[2][1] = tmp[2][1]; rotmat[2][2] = tmp[2][2];
}

//==========================================================================================================================

void OptimalTransformation::getRigidTransform(const double *a1, const double *a1_n, const double *b1, const double* b1_n,
		const double *a2, const double *a2_n, const double *b2, const double* b2_n, double** hommat) const
{
	// Some local variables
	double o1[3], o2[3], x1[3], x2[3], y1[3], y2[3], z1[3], z2[3], tmp1[3], tmp2[3], Ro1[3];
	Matrix invFrame1(3, 3), R(3, 3);

	// Compute the origins
	o1[0] = (a1[0] + b1[0])/2.0;
	o1[1] = (a1[1] + b1[1])/2.0;
	o1[2] = (a1[2] + b1[2])/2.0;

	o2[0] = (a2[0] + b2[0])/2.0;
	o2[1] = (a2[1] + b2[1])/2.0;
	o2[2] = (a2[2] + b2[2])/2.0;

	// Compute the x-axes
	Vector::diff(b1, a1, x1); Vector::normalize3(x1);
	Vector::diff(b2, a2, x2); Vector::normalize3(x2);
	// Compute the y-axes. First y-axis
	AnalyticGeometry::projectOnPlane3(a1_n, x1, tmp1); Vector::normalize3(tmp1);
	AnalyticGeometry::projectOnPlane3(b1_n, x1, tmp2); Vector::normalize3(tmp2);
	Vector::sum3(tmp1, tmp2, y1); Vector::normalize3(y1);
	// Second y-axis
	AnalyticGeometry::projectOnPlane3(a2_n, x2, tmp1); Vector::normalize3(tmp1);
	AnalyticGeometry::projectOnPlane3(b2_n, x2, tmp2); Vector::normalize3(tmp2);
	Vector::sum3(tmp1, tmp2, y2); Vector::normalize3(y2);
	// Compute the z-axes
	Vector::cross3(x1, y1, z1);
	Vector::cross3(x2, y2, z2);

	// 1. Invert [x1 y1 z1]
	invFrame1.m[0][0] = x1[0]; invFrame1.m[0][1] = x1[1]; invFrame1.m[0][2] = x1[2];
	invFrame1.m[1][0] = y1[0]; invFrame1.m[1][1] = y1[1]; invFrame1.m[1][2] = y1[2];
	invFrame1.m[2][0] = z1[0]; invFrame1.m[2][1] = z1[1]; invFrame1.m[2][2] = z1[2];
	// 2. Multiply
	Matrix::mult3x3(x2, y2, z2, invFrame1.getm(), R.m);

	// Construct the homogeneous 'out' matrix
	Matrix::copy3x3(R.getm(), hommat);
	// The rotated origin of the first coordinate frame
	Matrix::mult3x3(R.getm(), o1, Ro1);
	hommat[0][3] = o2[0] - Ro1[0];
	hommat[1][3] = o2[1] - Ro1[1];
	hommat[2][3] = o2[2] - Ro1[2];
	hommat[3][0] = hommat[3][1] = hommat[3][2] = 0.0; hommat[3][3] = 1.0;
}

//==========================================================================================================================

void OptimalTransformation::getRigidTransform(const double *a1, const double *a1_n, const double *b1, const double* b1_n,
		const double *a2, const double *a2_n, const double *b2, const double* b2_n, double* rig) const
{
	// Some local variables
	double o1[3], o2[3], x1[3], x2[3], y1[3], y2[3], z1[3], z2[3], tmp1[3], tmp2[3], Ro1[3];
	Matrix invFrame1(3, 3), rot(3, 3);

	// Compute the origins
	o1[0] = (a1[0] + b1[0])/2.0;
	o1[1] = (a1[1] + b1[1])/2.0;
	o1[2] = (a1[2] + b1[2])/2.0;

	o2[0] = (a2[0] + b2[0])/2.0;
	o2[1] = (a2[1] + b2[1])/2.0;
	o2[2] = (a2[2] + b2[2])/2.0;

	// Compute the x-axes
	Vector::diff(b1, a1, x1); Vector::normalize3(x1);
	Vector::diff(b2, a2, x2); Vector::normalize3(x2);
	// Compute the y-axes. First y-axis
	AnalyticGeometry::projectOnPlane3(a1_n, x1, tmp1); Vector::normalize3(tmp1);
	AnalyticGeometry::projectOnPlane3(b1_n, x1, tmp2); Vector::normalize3(tmp2);
	Vector::sum3(tmp1, tmp2, y1); Vector::normalize3(y1);
	// Second y-axis
	AnalyticGeometry::projectOnPlane3(a2_n, x2, tmp1); Vector::normalize3(tmp1);
	AnalyticGeometry::projectOnPlane3(b2_n, x2, tmp2); Vector::normalize3(tmp2);
	Vector::sum3(tmp1, tmp2, y2); Vector::normalize3(y2);
	// Compute the z-axes
	Vector::cross3(x1, y1, z1);
	Vector::cross3(x2, y2, z2);

	// 1. Invert [x1 y1 z1]
	invFrame1.m[0][0] = x1[0]; invFrame1.m[0][1] = x1[1]; invFrame1.m[0][2] = x1[2];
	invFrame1.m[1][0] = y1[0]; invFrame1.m[1][1] = y1[1]; invFrame1.m[1][2] = y1[2];
	invFrame1.m[2][0] = z1[0]; invFrame1.m[2][1] = z1[1]; invFrame1.m[2][2] = z1[2];
	// 2. Multiply
	Matrix::mult3x3(x2, y2, z2, invFrame1.getm(), rot.m);

	// Construct the homogeneous 'out' matrix
	Matrix::copy3x3(rot.getm(), rig);
	// The rotated origin of the first coordinate frame
	Matrix::mult3x3(rot.getm(), o1, Ro1);
	rig[9]  = o2[0] - Ro1[0];
	rig[10] = o2[1] - Ro1[1];
	rig[11] = o2[2] - Ro1[2];
}

//==========================================================================================================================
