#ifndef _VTKT_RANSFORM_H_
#define _VTKT_RANSFORM_H_

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkMatrixToHomogeneousTransform.h>
#include "../LinearAlgebra/Matrix.h"
#include <list>

using namespace std;
using namespace tum;

class VtkTransform
{
public:
	VtkTransform();
	virtual ~VtkTransform();

	inline static void double12ToHomVtkMath4x4(const double *T, vtkMatrix4x4* m);
	inline static void rigid12ToTransformer(const double *T, vtkTransformPolyDataFilter* out);

	/** Convert the 4x4 matrix 'mat' to a vtkMatrix4x4 object. */
	inline static void double4x4ToHomVtkMath4x4(const double **mat, vtkMatrix4x4* m);
	/** Convert the 4x4 matrix 'm' to a vtkTransformPolyDataFilter object. */
	inline static void mat4x4ToTransformer(const double **m, vtkTransformPolyDataFilter* out);

	/** %s is the scale and %rigid is a 12-vector containing a rotation (first 9 elements)
	  * and a translation (last 3 elements). */
	inline static void similarityToHomVtkMath4x4(double s, const double *rigid, vtkMatrix4x4* m);

	/** Multiplies points by the 3x3 matrix m. */
	static void mult(vtkPoints *points, const double **m);

	/** Multiplies the normals of 'polydata' by the 3x3 matrix 'm'. */
	static void multNormals3x3(vtkPolyData *polydata, const double **m);

	/** The first 9 elements of %T are interpreted as elements of a 3x3 rotation matrix R.
	  * The method multiplies the normals of %polydata with R. */
	static void multNormals9(vtkPolyData *polydata, const double *T);

	/** Computes the mean distance between all pairs of closest points. */
	static double compMeanClosestPointDist(vtkPoints* points);

	/** Extends the 'points' to homogoneous coordinates and multiplies them by 'mat'. */
	inline static void mult4x4(vtkPoints* points, const double **mat);
	/** Extends the 'points' to homogoneous coordinates and multiplies them by 'mat'. */
	inline static void mult4x4(vtkPoints* points, const double mat[4][4]);

	/** Extends the points 'in' to homogoneous coordinates and multiplies them by 'mat'. The result
	 * is saved in 'out' */
	inline static void mult4x4(vtkPoints* in, const double **mat, vtkPoints* out);

	/** The first 9 elements of %T will be interpreted as the elements of a 3x3 matrix A and
	  * the last 3 elements build the translation vector t. The method computes p = Ap + t for
	  * each point p in %points. */
	inline static void T12(vtkPoints *points, const double *T);

	/** Multiplies points by R and then adds T. */
	static void RT(vtkPoints *points, double R[3][3], double T[3]);

	/** Multiplies points by R (a 3x3 matrix) and then adds T. */
	static void RT(vtkPoints *points, double **R, double T[3]);

	/** Multiplies the points of polyData by R and adds T. */
	static void RT(vtkPolyData *polyData, double R[3][3], double T[3]){ VtkTransform::RT(polyData->GetPoints(), R, T);}

	/** Multiplies every point of 'src' by R, adds T and inserts the resulting point in 'dst'. */
	static void RT(vtkPoints* src, double R[3][3], double T[3], vtkPoints* dst);

	/** For every point p in %src do: insert sR*p + T in %dst. */
	static void sRT(vtkPoints* src, double s, double R[3][3], double T[3], vtkPoints* dst);

	/** Multiplies points by the matrix R. */
	static void R(vtkPoints *points, double R[3][3]);
	/** Multiplies the points of polyData by the matrix R. */
	static void R(vtkPolyData *polyData, double R[3][3]){ VtkTransform::R(polyData->GetPoints(), R);}

	/** Translates the points by T. */
	static void T(vtkPoints *points, double T[3]);
	static void T(vtkPoints *points, double x, double y, double z){ double t[3] = {x,y,z}; VtkTransform::T(points, t);}

	/** Scales points by s. */
	static void S(vtkPoints *points, double s);
	/** Scales the points of polyData by s. */
	inline static void S(vtkPolyData *polyData, double s){ VtkTransform::S(polyData->GetPoints(), s);}

	/** Saves the bounds of points in min and max. */
	static void getBounds(vtkPoints *points, double min[3], double max[3]);

	/** Rotates all points by u (in radians) about the x-axis. */
	static void rotateAboutX(double u, vtkPoints *points);
	/** Rotates all points by v (in radians) about the y-axis. */
	static void rotateAboutY(double v, vtkPoints *points);
	/** Rotates all points  by w (in radians) about the z-axis. */
	static void rotateAboutZ(double w, vtkPoints *points);
	/** Translates the points to the origin, performs scaling by factor 's' and translates the points back. */
	static void scaleAboutCentroid(vtkPoints* points, double s);

	/** Computes a rotation matrix and saves it in rotmat. This matrix describes a rotation (order is important!)
	  * by u about the x-axis, by v about the y-axis and by w about the z-axis. */
	static void getRotationMatrixXYZ(double u, double v, double w, double rotmat[3][3]);
	/** Computes a rotation matrix and saves it in rotmat. This matrix describes a rotation (order is important!)
	  * by w about the z-axis, by v about the y-axis and by u about the x-axis. */
	static void getRotationMatrixZYX(double u, double v, double w, double rotmat[3][3]);

	/** Computes a rotation matrix and saves it in rotmat. This matrix describes a rotation by u about the x-axis */
	static void getRotationMatrixX(double u, double rotmat[3][3]);
	/** Computes a rotation matrix and saves it in rotmat. This matrix describes a rotation by u about the y-axis */
	static void getRotationMatrixY(double u, double rotmat[3][3]);
	/** Computes a rotation matrix and saves it in rotmat. This matrix describes a rotation by u about the z-axis */
	static void getRotationMatrixZ(double u, double rotmat[3][3]);
	/** Computes a rotation matrix and saves it in rotmat. This matrix describes a rotation by u about the x-axis */
	static void getRotationMatrixX(double u, double** rotmat);
	/** Computes a rotation matrix and saves it in rotmat. This matrix describes a rotation by u about the y-axis */
	static void getRotationMatrixY(double u, double** rotmat);
	/** Computes a rotation matrix and saves it in rotmat. This matrix describes a rotation by u about the z-axis */
	static void getRotationMatrixZ(double u, double** rotmat);

	/** Computes the center of mass of points and saves it in com. */
	static void getCenterOfMass(vtkPoints *points, double com[3]);
	/** Rotates the cam about the (0, 0, 1) vector placed at the focal point of cam. */
	static void rotateCamAboutFocalPoint(vtkCamera *cam, double rad);

	/** Computes the centroid of 'points' and subtracts it from each point. */
	static void centerPoints(vtkPoints* points);

	/** Copies the points from src to dst. Note that this method inserts the points from src in dst, i.e., existing points
	 * in dst will NOT be deleted. */
	static void copy(vtkPoints* src, vtkPoints* dst);
	/** Copies the points from 'src' with ids 'src_ids' to 'dst'. Note that this method inserts the points in 'dst',
	 * i.e., existing points in 'dst' will NOT be deleted. */
	static void copy(vtkPoints* src, list<int>& src_ids, vtkPoints* dst);

	/** Returns the mean distance between the corresponding points in 'pts1' and 'pts2'. */
	static double computeMeanDist(vtkPoints* pts1, vtkPoints* pts2);

	/** Computes the root mean squared error between the corresponding points sets 'pts1' and 'pts2'.
	  * RMS(A, B) = sqrt(1/N * {sum over ||a_i-b_i||^2}), where A and B are point sets consisting of
	  * the points a_1, ..., a_N and b_1, ..., b_N, respectively. */
	static double RMS(vtkPoints* pts1, vtkPoints* pts2);
};

//=== inline methods ==========================================================================================================

inline void VtkTransform::double12ToHomVtkMath4x4(const double *T, vtkMatrix4x4* m)
{
	m->SetElement(0,0, T[0]); m->SetElement(0,1, T[1]); m->SetElement(0,2, T[2]); m->SetElement(0,3, T[9]);
	m->SetElement(1,0, T[3]); m->SetElement(1,1, T[4]); m->SetElement(1,2, T[5]); m->SetElement(1,3, T[10]);
	m->SetElement(2,0, T[6]); m->SetElement(2,1, T[7]); m->SetElement(2,2, T[8]); m->SetElement(2,3, T[11]);
	m->SetElement(3,0, 0.0);  m->SetElement(3,1, 0.0);  m->SetElement(3,2, 0.0);  m->SetElement(3,3, 1.0);
}

//=============================================================================================================================

inline void VtkTransform::rigid12ToTransformer(const double *T, vtkTransformPolyDataFilter* out)
{
	vtkMatrix4x4 *mat = vtkMatrix4x4::New();
	// Convert the 12-vector to a vtk matrix.
	VtkTransform::double12ToHomVtkMath4x4(T, mat);

	// Create a vtk transform object
	vtkMatrixToHomogeneousTransform *transform = vtkMatrixToHomogeneousTransform::New();
	  transform->SetInput(mat);

	// Initialize the transformer
	out->SetTransform(transform);
	out->Update();

	// Cleanup
	mat->Delete();
	transform->Delete();
}

//=============================================================================================================================

inline void VtkTransform::double4x4ToHomVtkMath4x4(const double **mat, vtkMatrix4x4* m)
{
	m->SetElement(0,0, mat[0][0]); m->SetElement(0,1, mat[0][1]); m->SetElement(0,2, mat[0][2]); m->SetElement(0,3, mat[0][3]);
	m->SetElement(1,0, mat[1][0]); m->SetElement(1,1, mat[1][1]); m->SetElement(1,2, mat[1][2]); m->SetElement(1,3, mat[1][3]);
	m->SetElement(2,0, mat[2][0]); m->SetElement(2,1, mat[2][1]); m->SetElement(2,2, mat[2][2]); m->SetElement(2,3, mat[2][3]);
	m->SetElement(3,0, mat[3][0]); m->SetElement(3,1, mat[3][1]); m->SetElement(3,2, mat[3][2]); m->SetElement(3,3, mat[3][3]);
}

//=============================================================================================================================

inline void VtkTransform::mat4x4ToTransformer(const double **m, vtkTransformPolyDataFilter* out)
{
	vtkMatrix4x4 *mat = vtkMatrix4x4::New();
	// Convert the 4x4-matrix to a vtk matrix.
	VtkTransform::double4x4ToHomVtkMath4x4(m, mat);

	// Create a vtk transform object
	vtkMatrixToHomogeneousTransform *transform = vtkMatrixToHomogeneousTransform::New();
	  transform->SetInput(mat);

	// Initialize the transformer
	out->SetTransform(transform);
	out->Update();

	// Cleanup
	mat->Delete();
	transform->Delete();
}

//=============================================================================================================================

inline void VtkTransform::similarityToHomVtkMath4x4(double s, const double *rigid, vtkMatrix4x4* m)
{
	m->SetElement(0,0, s*rigid[0]); m->SetElement(0,1, s*rigid[1]); m->SetElement(0,2, s*rigid[2]); m->SetElement(0,3, rigid[9]);
	m->SetElement(1,0, s*rigid[3]); m->SetElement(1,1, s*rigid[4]); m->SetElement(1,2, s*rigid[5]); m->SetElement(1,3, rigid[10]);
	m->SetElement(2,0, s*rigid[6]); m->SetElement(2,1, s*rigid[7]); m->SetElement(2,2, s*rigid[8]); m->SetElement(2,3, rigid[11]);
	m->SetElement(3,0, 0.0);        m->SetElement(3,1, 0.0);        m->SetElement(3,2, 0.0);        m->SetElement(3,3, 1.0);
}

//=============================================================================================================================

inline void VtkTransform::mult4x4(vtkPoints* points, const double **mat)
{
	double p1[4], p2[4];
	int i, numOfPoints = points->GetNumberOfPoints();

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		points->GetPoint(i, p1);
		p1[3] = p2[3] = 1.0;
		Matrix::mult4x4(mat, p1, p2);
		points->SetPoint(i, p2);
	}
}

//=============================================================================================================================

inline void VtkTransform::mult4x4(vtkPoints* points, const double mat[4][4])
{
	double p1[3], p2[3];
	int i, numOfPoints = points->GetNumberOfPoints();

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		points->GetPoint(i, p1);
		p2[0] = mat[0][0]*p1[0] + mat[0][1]*p1[1] + mat[0][2]*p1[2];
		p2[1] = mat[1][0]*p1[0] + mat[1][1]*p1[1] + mat[1][2]*p1[2];
		p2[2] = mat[2][0]*p1[0] + mat[2][1]*p1[1] + mat[2][2]*p1[2];
		p2[0] += mat[0][3];
		p2[1] += mat[1][3];
		p2[2] += mat[2][3];
		points->SetPoint(i, p2);
	}
}

//=============================================================================================================================

inline void VtkTransform::mult4x4(vtkPoints* in, const double **mat, vtkPoints* out)
{
	double p1[4], p2[4];
	int i, numOfPoints = in->GetNumberOfPoints();

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		in->GetPoint(i, p1);
		p1[3] = p2[3] = 1.0;
		Matrix::mult4x4(mat, p1, p2);
		out->InsertNextPoint(p2);
	}
}

//===============================================================================================================================

inline void VtkTransform::T12(vtkPoints *points, const double *T)
{
	int i, nOfPoints = points->GetNumberOfPoints();
	double p1[3], p2[3];

	for ( i = 0 ; i < nOfPoints ; ++i )
	{
		points->GetPoint(i, p1);
		// Rotate and translate
		p2[0] = p1[0]*T[0] + p1[1]*T[1] + p1[2]*T[2] + T[9];
		p2[1] = p1[0]*T[3] + p1[1]*T[4] + p1[2]*T[5] + T[10];
		p2[2] = p1[0]*T[6] + p1[1]*T[7] + p1[2]*T[8] + T[11];
		points->SetPoint(i, p2);
	}
}

//=============================================================================================================================

#endif /*_VTK_TRANSFORM_H_*/
