/*
 * Matrix.h
 *
 *  Created on: Dec 6, 2010
 *      Author: papazov
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include "eigen3x3/dsyevh3.h"
#include "Vector.h"
#include <cmath>
#include <cstdio>

namespace tum
{

#define M_SQR(x) ((x)*(x))

/** Very simple matrix operations like multiplication, transposition, ... */

//==================================================================================================================

/** Allocates memory for a matrix with 'rows' x 'cols' double elements. */
inline double** mat_alloc(int rows, int cols)
{
	double** m = new double*[rows];
	for ( int i = 0 ; i < rows ; ++i )
		m[i] = new double[cols];
	return m;
}

//==================================================================================================================

/** Frees the memory for %mat previously allocated with %mat_alloc(). */
inline void mat_dealloc(double** mat, int rows)
{
	for ( int i = 0 ; i < rows ; ++i )
		delete[] mat[i];
	delete[] mat;
}

//==================================================================================================================

inline void mat_print4x4(const double** mat, FILE* stream)
{
	fprintf(stream, "%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n",
			mat[0][0], mat[0][1], mat[0][2], mat[0][3],
			mat[1][0], mat[1][1], mat[1][2], mat[1][3],
			mat[2][0], mat[2][1], mat[2][2], mat[2][3],
			mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
	fflush(stream);
}

//==================================================================================================================

inline void mat_print9(const double* mat, FILE* stream = stdout)
{
	fprintf(stream, "%lf %lf %lf\n%lf %lf %lf\n%lf %lf %lf\n",
			mat[0], mat[1], mat[2],
			mat[3], mat[4], mat[5],
			mat[6], mat[7], mat[8]);
	fflush(stream);
}

//==================================================================================================================

inline void mat_print3x3(const double** mat, FILE* stream = stdout)
{
	fprintf(stream, "%lf %lf %lf\n%lf %lf %lf\n%lf %lf %lf\n",
			mat[0][0], mat[0][1], mat[0][2],
			mat[1][0], mat[1][1], mat[1][2],
			mat[2][0], mat[2][1], mat[2][2]);
	fflush(stream);
}

//==================================================================================================================

inline void mat_print3x3(const double mat[3][3], FILE* stream = stdout)
{
	fprintf(stream, "%.9lf %.9lf %.9lf\n%.9lf %.9lf %.9lf\n%.9lf %.9lf %.9lf\n",
			mat[0][0], mat[0][1], mat[0][2],
			mat[1][0], mat[1][1], mat[1][2],
			mat[2][0], mat[2][1], mat[2][2]);
	fflush(stream);
}

//==================================================================================================================

inline void mat_copy3x3(const double** src, double dst[3][3])
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void mat_copy3x3(double src[3][3], double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void mat_copy3x3(const double src[3][3], double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void mat_copy3x3(const double** src, double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void mat_copy3x3(const double** src, float* dst)
{
	dst[0] = (float)src[0][0]; dst[1] = (float)src[0][1]; dst[2] = (float)src[0][2];
	dst[3] = (float)src[1][0]; dst[4] = (float)src[1][1]; dst[5] = (float)src[1][2];
	dst[6] = (float)src[2][0]; dst[7] = (float)src[2][1]; dst[8] = (float)src[2][2];
}

//==================================================================================================================

inline void mat_copy3x3(const double** src, double* dst)
{
	dst[0] = src[0][0]; dst[1] = src[0][1]; dst[2] = src[0][2];
	dst[3] = src[1][0]; dst[4] = src[1][1]; dst[5] = src[1][2];
	dst[6] = src[2][0]; dst[7] = src[2][1]; dst[8] = src[2][2];
}

//==================================================================================================================

inline void mat_copy3x3(const double src[3][3], double* dst)
{
	dst[0] = src[0][0]; dst[1] = src[0][1]; dst[2] = src[0][2];
	dst[3] = src[1][0]; dst[4] = src[1][1]; dst[5] = src[1][2];
	dst[6] = src[2][0]; dst[7] = src[2][1]; dst[8] = src[2][2];
}

//==================================================================================================================

inline void mat_copy3x3(const double src[3][3], float* dst)
{
	dst[0] = (float)src[0][0]; dst[1] = (float)src[0][1]; dst[2] = (float)src[0][2];
	dst[3] = (float)src[1][0]; dst[4] = (float)src[1][1]; dst[5] = (float)src[1][2];
	dst[6] = (float)src[2][0]; dst[7] = (float)src[2][1]; dst[8] = (float)src[2][2];
}

//==================================================================================================================

inline void mat_copy3x3(const float* src, double dst[3][3])
{
	dst[0][0] = (double)src[0]; dst[0][1] = (double)src[1]; dst[0][2] = (double)src[2];
	dst[1][0] = (double)src[3]; dst[1][1] = (double)src[4]; dst[1][2] = (double)src[5];
	dst[2][0] = (double)src[6]; dst[2][1] = (double)src[7]; dst[2][2] = (double)src[8];
}

//==================================================================================================================

inline void mat_copy3x3(const double* src, double dst[3][3])
{
	dst[0][0] = src[0]; dst[0][1] = src[1]; dst[0][2] = src[2];
	dst[1][0] = src[3]; dst[1][1] = src[4]; dst[1][2] = src[5];
	dst[2][0] = src[6]; dst[2][1] = src[7]; dst[2][2] = src[8];
}

//==================================================================================================================

inline void mat_copy4x4(double** src, double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2]; dst[0][3] = src[0][3];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2]; dst[1][3] = src[1][3];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2]; dst[2][3] = src[2][3];
	dst[3][0] = src[3][0]; dst[3][1] = src[3][1]; dst[3][2] = src[3][2]; dst[3][3] = src[3][3];
}

//==================================================================================================================

inline void mat_copy_rigid_transform_to4x4(const double* src, double** hommat4x4)
{
	// The rotation
	hommat4x4[0][0] = src[0]; hommat4x4[0][1] = src[1]; hommat4x4[0][2] = src[2];
	hommat4x4[1][0] = src[3]; hommat4x4[1][1] = src[4]; hommat4x4[1][2] = src[5];
	hommat4x4[2][0] = src[6]; hommat4x4[2][1] = src[7]; hommat4x4[2][2] = src[8];
	// The translation
	hommat4x4[0][3] = src[9];
	hommat4x4[1][3] = src[10];
	hommat4x4[2][3] = src[11];
	// The last row
	hommat4x4[3][0] = hommat4x4[3][1] = hommat4x4[3][2] = 0.0; hommat4x4[3][3] = 1.0;
}

//==================================================================================================================

inline void mat_copy_rigid_transform_to4x4(const double* src, double hommat4x4[4][4])
{
	// The rotation
	hommat4x4[0][0] = src[0]; hommat4x4[0][1] = src[1]; hommat4x4[0][2] = src[2];
	hommat4x4[1][0] = src[3]; hommat4x4[1][1] = src[4]; hommat4x4[1][2] = src[5];
	hommat4x4[2][0] = src[6]; hommat4x4[2][1] = src[7]; hommat4x4[2][2] = src[8];
	// The translation
	hommat4x4[0][3] = src[9];
	hommat4x4[1][3] = src[10];
	hommat4x4[2][3] = src[11];
	// The last row
	hommat4x4[3][0] = hommat4x4[3][1] = hommat4x4[3][2] = 0.0; hommat4x4[3][3] = 1.0;
}

//==================================================================================================================

inline void mat_invert_rigid_transform4x4(const double m[4][4], double mi[4][4])
{
	// The rotational part
	mi[0][0] = m[0][0]; mi[0][1] = m[1][0]; mi[0][2] = m[2][0];
	mi[1][0] = m[0][1]; mi[1][1] = m[1][1]; mi[1][2] = m[2][1];
	mi[2][0] = m[0][2]; mi[2][1] = m[1][2]; mi[2][2] = m[2][2];
	// The translational part
	mi[0][3] = -(m[0][0]*m[0][3] + m[1][0]*m[1][3] + m[2][0]*m[2][3]);
	mi[1][3] = -(m[0][1]*m[0][3] + m[1][1]*m[1][3] + m[2][1]*m[2][3]);
	mi[2][3] = -(m[0][2]*m[0][3] + m[1][2]*m[1][3] + m[2][2]*m[2][3]);
	// The last row
	mi[3][0] = mi[3][1] = mi[3][2] = 0.0; mi[3][3] = 1.0;
}

//==================================================================================================================

inline void mat_invert_rigid_transform4x4(const double** m, double** mi)
{
	// The rotational part
	mi[0][0] = m[0][0]; mi[0][1] = m[1][0]; mi[0][2] = m[2][0];
	mi[1][0] = m[0][1]; mi[1][1] = m[1][1]; mi[1][2] = m[2][1];
	mi[2][0] = m[0][2]; mi[2][1] = m[1][2]; mi[2][2] = m[2][2];
	// The translational part
	mi[0][3] = -(m[0][0]*m[0][3] + m[1][0]*m[1][3] + m[2][0]*m[2][3]);
	mi[1][3] = -(m[0][1]*m[0][3] + m[1][1]*m[1][3] + m[2][1]*m[2][3]);
	mi[2][3] = -(m[0][2]*m[0][3] + m[1][2]*m[1][3] + m[2][2]*m[2][3]);
	// The last row
	mi[3][0] = mi[3][1] = mi[3][2] = 0.0; mi[3][3] = 1.0;
}

//==================================================================================================================

inline void mat_copy_hom4x4_to_vec12(const double** hommat4x4, double* dst)
{
	// The rotation
	dst[0] = hommat4x4[0][0]; dst[1] = hommat4x4[0][1]; dst[2] = hommat4x4[0][2];
	dst[3] = hommat4x4[1][0]; dst[4] = hommat4x4[1][1]; dst[5] = hommat4x4[1][2];
	dst[6] = hommat4x4[2][0]; dst[7] = hommat4x4[2][1]; dst[8] = hommat4x4[2][2];
	// The translation
	dst[9]  = hommat4x4[0][3];
	dst[10] = hommat4x4[1][3];
	dst[11] = hommat4x4[2][3];
}

//==================================================================================================================

inline void mat_mult3xN(const double m1[3][3], double** m2, int numOfCols)
{
	double tmp[3];
	for ( int i = 0 ; i < numOfCols ; ++i )
	{
		tmp[0] = m1[0][0]*m2[0][i] + m1[0][1]*m2[1][i] + m1[0][2]*m2[2][i];
		tmp[1] = m1[1][0]*m2[0][i] + m1[1][1]*m2[1][i] + m1[1][2]*m2[2][i];
		tmp[2] = m1[2][0]*m2[0][i] + m1[2][1]*m2[1][i] + m1[2][2]*m2[2][i];
		m2[0][i] = tmp[0];
		m2[1][i] = tmp[1];
		m2[2][i] = tmp[2];
	}
}

//==================================================================================================================

inline void mat_mult4x4(const double** mat, const double* v, double* out)
{
	out[0] = mat[0][0]*v[0] + mat[0][1]*v[1] + mat[0][2]*v[2] + mat[0][3]*v[3];
	out[1] = mat[1][0]*v[0] + mat[1][1]*v[1] + mat[1][2]*v[2] + mat[1][3]*v[3];
	out[2] = mat[2][0]*v[0] + mat[2][1]*v[1] + mat[2][2]*v[2] + mat[2][3]*v[3];
	out[3] = mat[3][0]*v[0] + mat[3][1]*v[1] + mat[3][2]*v[2] + mat[3][3]*v[3];
}

//==================================================================================================================

/** Multiplies the first row by 'x', the second by 'y' and the third by 'z' */
inline void mat_mult_rows3x3(double mat[3][3], double x, double y, double z)
{
	mat[0][0] *= x; mat[0][1] *= x; mat[0][2] *= x;
	mat[1][0] *= y; mat[1][1] *= y; mat[1][2] *= y;
	mat[2][0] *= z; mat[2][1] *= z; mat[2][2] *= z;
}

//==================================================================================================================

inline void mat_copy3x3(const double* x, const double* y, const double* z, double** dst)
{
	dst[0][0] = x[0]; dst[0][1] = y[0]; dst[0][2] = z[0];
	dst[1][0] = x[1]; dst[1][1] = y[1]; dst[1][2] = z[1];
	dst[2][0] = x[2]; dst[2][1] = y[2]; dst[2][2] = z[2];
}

//==================================================================================================================

inline void mat_copy3x3(const double src[3][3], double dst[3][3])
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void mat_transpose3x3(const double src[3][3], double dst[3][3])
{
	dst[0][0] = src[0][0]; dst[0][1] = src[1][0]; dst[0][2] = src[2][0];
	dst[1][0] = src[0][1]; dst[1][1] = src[1][1]; dst[1][2] = src[2][1];
	dst[2][0] = src[0][2]; dst[2][1] = src[1][2]; dst[2][2] = src[2][2];
}

//==================================================================================================================

template<class T>
inline void mat_transpose9(const T* src, T* dst)
{
	dst[0] = src[0]; dst[3] = src[1]; dst[6] = src[2];
	dst[1] = src[3]; dst[4] = src[4]; dst[7] = src[5];
	dst[2] = src[6]; dst[5] = src[7]; dst[8] = src[8];
}

//==================================================================================================================

inline void mat_transpose3x3(const double** src, double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[1][0]; dst[0][2] = src[2][0];
	dst[1][0] = src[0][1]; dst[1][1] = src[1][1]; dst[1][2] = src[2][1];
	dst[2][0] = src[0][2]; dst[2][1] = src[1][2]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void mat_transpose4x4(const double** src, double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[1][0]; dst[0][2] = src[2][0]; dst[0][3] = src[3][0];
	dst[1][0] = src[0][1]; dst[1][1] = src[1][1]; dst[1][2] = src[2][1]; dst[1][3] = src[3][1];
	dst[2][0] = src[0][2]; dst[2][1] = src[1][2]; dst[2][2] = src[2][2]; dst[2][3] = src[3][2];
	dst[3][0] = src[0][3]; dst[3][1] = src[1][3]; dst[3][2] = src[2][3]; dst[3][3] = src[3][3];
}

//==================================================================================================================

template<class T>
inline void mat_mult_vec_with_mat9(const T* mat, const T* u, T* out)
{
	out[0] = mat[0]*u[0] + mat[1]*u[1] + mat[2]*u[2];
	out[1] = mat[3]*u[0] + mat[4]*u[1] + mat[5]*u[2];
	out[2] = mat[6]*u[0] + mat[7]*u[1] + mat[8]*u[2];
}

//==================================================================================================================

template<class T>
inline void mat_mult9(const T* m1, const T* m2, T* out)
{
	out[0] = m1[0]*m2[0] + m1[1]*m2[3] + m1[2]*m2[6];
	out[1] = m1[0]*m2[1] + m1[1]*m2[4] + m1[2]*m2[7];
	out[2] = m1[0]*m2[2] + m1[1]*m2[5] + m1[2]*m2[8];

	out[3] = m1[3]*m2[0] + m1[4]*m2[3] + m1[5]*m2[6];
	out[4] = m1[3]*m2[1] + m1[4]*m2[4] + m1[5]*m2[7];
	out[5] = m1[3]*m2[2] + m1[4]*m2[5] + m1[5]*m2[8];

	out[6] = m1[6]*m2[0] + m1[7]*m2[3] + m1[8]*m2[6];
	out[7] = m1[6]*m2[1] + m1[7]*m2[4] + m1[8]*m2[7];
	out[8] = m1[6]*m2[2] + m1[7]*m2[5] + m1[8]*m2[8];
}

//==================================================================================================================

/** ATB = A^T * B */
template<class T>
inline void mat_mult_AT_B_9(const T* A, const T* B, T* ATB)
{
	ATB[0] = A[0]*B[0] + A[3]*B[3] + A[6]*B[6];
	ATB[1] = A[0]*B[1] + A[3]*B[4] + A[6]*B[7];
	ATB[2] = A[0]*B[2] + A[3]*B[5] + A[6]*B[8];

	ATB[3] = A[1]*B[0] + A[4]*B[3] + A[7]*B[6];
	ATB[4] = A[1]*B[1] + A[4]*B[4] + A[7]*B[7];
	ATB[5] = A[1]*B[2] + A[4]*B[5] + A[7]*B[8];

	ATB[6] = A[2]*B[0] + A[5]*B[3] + A[8]*B[6];
	ATB[7] = A[2]*B[1] + A[5]*B[4] + A[8]*B[7];
	ATB[8] = A[2]*B[2] + A[5]*B[5] + A[8]*B[8];
}

//==================================================================================================================

/** ABT = A * B^T */
template<class T>
inline void mat_mult_A_BT_9(const T* A, const T* B, T* ABT)
{
	ABT[0] = A[0]*B[0] + A[1]*B[1] + A[2]*B[2];
	ABT[1] = A[0]*B[3] + A[1]*B[4] + A[2]*B[5];
	ABT[2] = A[0]*B[6] + A[1]*B[7] + A[2]*B[8];

	ABT[3] = A[3]*B[0] + A[4]*B[1] + A[5]*B[2];
	ABT[4] = A[3]*B[3] + A[4]*B[4] + A[5]*B[5];
	ABT[5] = A[3]*B[6] + A[4]*B[7] + A[5]*B[8];

	ABT[6] = A[6]*B[0] + A[7]*B[1] + A[8]*B[2];
	ABT[7] = A[6]*B[3] + A[7]*B[4] + A[8]*B[5];
	ABT[8] = A[6]*B[6] + A[7]*B[7] + A[8]*B[8];
}

//==================================================================================================================

/** out = M*M^T. out and M have to be at least 9-arrays. */
template<class T>
inline void mat_mult_MMT_9(const T* M, T* out)
{
	out[0] = M[0]*M[0] + M[1]*M[1] + M[2]*M[2];
	out[1] = M[0]*M[3] + M[1]*M[4] + M[2]*M[5];
	out[2] = M[0]*M[6] + M[1]*M[7] + M[2]*M[8];

	out[3] = out[1];
	out[4] = M[3]*M[3] + M[4]*M[4] + M[5]*M[5];
	out[5] = M[3]*M[6] + M[4]*M[7] + M[5]*M[8];

	out[6] = out[2];
	out[7] = out[5];
	out[8] = M[6]*M[6] + M[7]*M[7] + M[8]*M[8];
}

//==================================================================================================================

/** out = M^T*M. out and M have to be at least 9-arrays. */
template<class T>
inline void mat_mult_MTM_9(const T* M, T* out)
{
	out[0] = M[0]*M[0] + M[3]*M[3] + M[6]*M[6];
	out[1] = M[0]*M[1] + M[3]*M[4] + M[6]*M[7];
	out[2] = M[0]*M[2] + M[3]*M[5] + M[6]*M[8];

	out[3] = out[1];
	out[4] = M[1]*M[1] + M[4]*M[4] + M[7]*M[7];
	out[5] = M[1]*M[2] + M[4]*M[5] + M[7]*M[8];

	out[6] = out[2];
	out[7] = out[5];
	out[8] = M[2]*M[2] + M[5]*M[5] + M[8]*M[8];
}

//==================================================================================================================

inline void mat_mult3x3(const double mat[3][3], const double v[3], double out[3])
{
	out[0] = v[0]*mat[0][0] + v[1]*mat[0][1] + v[2]*mat[0][2];
	out[1] = v[0]*mat[1][0] + v[1]*mat[1][1] + v[2]*mat[1][2];
	out[2] = v[0]*mat[2][0] + v[1]*mat[2][1] + v[2]*mat[2][2];
}

//==================================================================================================================

inline void mat_mult3x3(const double mat[3][3], double x, double y, double z, double out[3])
{
	out[0] = x*mat[0][0] + y*mat[0][1] + z*mat[0][2];
	out[1] = x*mat[1][0] + y*mat[1][1] + z*mat[1][2];
	out[2] = x*mat[2][0] + y*mat[2][1] + z*mat[2][2];
}

//==================================================================================================================

inline void mat_mult3x3(const double** mat, const double* v, double* out)
{
	out[0] = v[0]*mat[0][0] + v[1]*mat[0][1] + v[2]*mat[0][2];
	out[1] = v[0]*mat[1][0] + v[1]*mat[1][1] + v[2]*mat[1][2];
	out[2] = v[0]*mat[2][0] + v[1]*mat[2][1] + v[2]*mat[2][2];
}

//==================================================================================================================

inline void mat_mult3x3(const double m1[3][3], const double m2[3][3], double out[3][3])
{
	out[0][0] = m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0] + m1[0][2]*m2[2][0];
	out[0][1] = m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1] + m1[0][2]*m2[2][1];
	out[0][2] = m1[0][0]*m2[0][2] + m1[0][1]*m2[1][2] + m1[0][2]*m2[2][2];

	out[1][0] = m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0] + m1[1][2]*m2[2][0];
	out[1][1] = m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1] + m1[1][2]*m2[2][1];
	out[1][2] = m1[1][0]*m2[0][2] + m1[1][1]*m2[1][2] + m1[1][2]*m2[2][2];

	out[2][0] = m1[2][0]*m2[0][0] + m1[2][1]*m2[1][0] + m1[2][2]*m2[2][0];
	out[2][1] = m1[2][0]*m2[0][1] + m1[2][1]*m2[1][1] + m1[2][2]*m2[2][1];
	out[2][2] = m1[2][0]*m2[0][2] + m1[2][1]*m2[1][2] + m1[2][2]*m2[2][2];
}

//==================================================================================================================

inline void mat_mult3x3(const double** m1, const double** m2, double** out)
{
	out[0][0] = m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0] + m1[0][2]*m2[2][0];
	out[0][1] = m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1] + m1[0][2]*m2[2][1];
	out[0][2] = m1[0][0]*m2[0][2] + m1[0][1]*m2[1][2] + m1[0][2]*m2[2][2];

	out[1][0] = m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0] + m1[1][2]*m2[2][0];
	out[1][1] = m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1] + m1[1][2]*m2[2][1];
	out[1][2] = m1[1][0]*m2[0][2] + m1[1][1]*m2[1][2] + m1[1][2]*m2[2][2];

	out[2][0] = m1[2][0]*m2[0][0] + m1[2][1]*m2[1][0] + m1[2][2]*m2[2][0];
	out[2][1] = m1[2][0]*m2[0][1] + m1[2][1]*m2[1][1] + m1[2][2]*m2[2][1];
	out[2][2] = m1[2][0]*m2[0][2] + m1[2][1]*m2[1][2] + m1[2][2]*m2[2][2];
}

//==================================================================================================================

inline void mat_mult3x3(const double* x, const double* y, const double* z, const double** m, double** out)
{
	out[0][0] = x[0]*m[0][0] + y[0]*m[1][0] + z[0]*m[2][0];
	out[0][1] = x[0]*m[0][1] + y[0]*m[1][1] + z[0]*m[2][1];
	out[0][2] = x[0]*m[0][2] + y[0]*m[1][2] + z[0]*m[2][2];

	out[1][0] = x[1]*m[0][0] + y[1]*m[1][0] + z[1]*m[2][0];
	out[1][1] = x[1]*m[0][1] + y[1]*m[1][1] + z[1]*m[2][1];
	out[1][2] = x[1]*m[0][2] + y[1]*m[1][2] + z[1]*m[2][2];

	out[2][0] = x[2]*m[0][0] + y[2]*m[1][0] + z[2]*m[2][0];
	out[2][1] = x[2]*m[0][1] + y[2]*m[1][1] + z[2]*m[2][1];
	out[2][2] = x[2]*m[0][2] + y[2]*m[1][2] + z[2]*m[2][2];
}

//==================================================================================================================

inline void mat_mult3x3(double m[3][3], double s)
{
	m[0][0] *= s; m[0][1] *= s; m[0][2] *= s;
	m[1][0] *= s; m[1][1] *= s; m[1][2] *= s;
	m[2][0] *= s; m[2][1] *= s; m[2][2] *= s;
}

//==================================================================================================================

/** out = a*b^T. */
inline void mat_tensor_product3x3(const double* a, const double* b, double out[3][3])
{
	out[0][0] = a[0]*b[0]; out[0][1] = a[0]*b[1]; out[0][2] = a[0]*b[2];
	out[1][0] = a[1]*b[0]; out[1][1] = a[1]*b[1]; out[1][2] = a[1]*b[2];
	out[2][0] = a[2]*b[0]; out[2][1] = a[2]*b[1]; out[2][2] = a[2]*b[2];
}

//==================================================================================================================

/** out = a*b^T. */
inline void mat_tensor_product9(const double* a, const double* b, double* out)
{
	out[0] = a[0]*b[0]; out[1] = a[0]*b[1]; out[2] = a[0]*b[2];
	out[3] = a[1]*b[0]; out[4] = a[1]*b[1]; out[5] = a[1]*b[2];
	out[6] = a[2]*b[0]; out[7] = a[2]*b[1]; out[8] = a[2]*b[2];
}

//==================================================================================================================

/** Assumes u = (x, y, z) and computes out = u*u^T.*/
inline void mat_tensor_product3x3(double x, double y, double z, double out[3][3])
{
	out[0][0] = x*x; out[0][1] = x*y; out[0][2] = x*z;
	out[1][0] = y*x; out[1][1] = y*y; out[1][2] = y*z;
	out[2][0] = z*x; out[2][1] = z*y; out[2][2] = z*z;
}

//==================================================================================================================

inline void mat_mult4x4(const double** m1, const double** m2, double** out)
{
	out[0][0] = m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0] + m1[0][2]*m2[2][0] + m1[0][3]*m2[3][0];
	out[0][1] = m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1] + m1[0][2]*m2[2][1] + m1[0][3]*m2[3][1];
	out[0][2] = m1[0][0]*m2[0][2] + m1[0][1]*m2[1][2] + m1[0][2]*m2[2][2] + m1[0][3]*m2[3][2];
	out[0][3] = m1[0][0]*m2[0][3] + m1[0][1]*m2[1][3] + m1[0][2]*m2[2][3] + m1[0][3]*m2[3][3];

	out[1][0] = m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0] + m1[1][2]*m2[2][0] + m1[1][3]*m2[3][0];
	out[1][1] = m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1] + m1[1][2]*m2[2][1] + m1[1][3]*m2[3][1];
	out[1][2] = m1[1][0]*m2[0][2] + m1[1][1]*m2[1][2] + m1[1][2]*m2[2][2] + m1[1][3]*m2[3][2];
	out[1][3] = m1[1][0]*m2[0][3] + m1[1][1]*m2[1][3] + m1[1][2]*m2[2][3] + m1[1][3]*m2[3][3];

	out[2][0] = m1[2][0]*m2[0][0] + m1[2][1]*m2[1][0] + m1[2][2]*m2[2][0] + m1[2][3]*m2[3][0];
	out[2][1] = m1[2][0]*m2[0][1] + m1[2][1]*m2[1][1] + m1[2][2]*m2[2][1] + m1[2][3]*m2[3][1];
	out[2][2] = m1[2][0]*m2[0][2] + m1[2][1]*m2[1][2] + m1[2][2]*m2[2][2] + m1[2][3]*m2[3][2];
	out[2][3] = m1[2][0]*m2[0][3] + m1[2][1]*m2[1][3] + m1[2][2]*m2[2][3] + m1[2][3]*m2[3][3];

	out[3][0] = m1[3][0]*m2[0][0] + m1[3][1]*m2[1][0] + m1[3][2]*m2[2][0] + m1[3][3]*m2[3][0];
	out[3][1] = m1[3][0]*m2[0][1] + m1[3][1]*m2[1][1] + m1[3][2]*m2[2][1] + m1[3][3]*m2[3][1];
	out[3][2] = m1[3][0]*m2[0][2] + m1[3][1]*m2[1][2] + m1[3][2]*m2[2][2] + m1[3][3]*m2[3][2];
	out[3][3] = m1[3][0]*m2[0][3] + m1[3][1]*m2[1][3] + m1[3][2]*m2[2][3] + m1[3][3]*m2[3][3];
}

//==================================================================================================================

inline void mat_set3x3(double m[3][3], double s)
{
	m[0][0] = m[0][1] = m[0][2] =
	m[1][0] = m[1][1] = m[1][2] =
	m[2][0] = m[2][1] = m[2][2] = s;
}

//==================================================================================================================

inline void mat_add3x3(double res[3][3], const double m[3][3])
{
	res[0][0] += m[0][0]; res[0][1] += m[0][1]; res[0][2] += m[0][2];
	res[1][0] += m[1][0]; res[1][1] += m[1][1]; res[1][2] += m[1][2];
	res[2][0] += m[2][0]; res[2][1] += m[2][1]; res[2][2] += m[2][2];
}

//==================================================================================================================

inline void mat_sub3x3(double res[3][3], const double m[3][3])
{
	res[0][0] -= m[0][0]; res[0][1] -= m[0][1]; res[0][2] -= m[0][2];
	res[1][0] -= m[1][0]; res[1][1] -= m[1][1]; res[1][2] -= m[1][2];
	res[2][0] -= m[2][0]; res[2][1] -= m[2][1]; res[2][2] -= m[2][2];
}

//==================================================================================================================

inline void mat_sub9(double* res, const double* m)
{
	res[0] -= m[0]; res[1] -= m[1]; res[2] -= m[2];
	res[3] -= m[3]; res[4] -= m[4]; res[5] -= m[5];
	res[6] -= m[6]; res[7] -= m[7]; res[8] -= m[8];
}

//==================================================================================================================

inline void mat_mult2x2(double **m1, double **m2, double **out)
{
	out[0][0] = m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0];
	out[0][1] = m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1];
	out[1][0] = m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0];
	out[1][1] = m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1];
}

//==================================================================================================================

inline void mat_mult2x2(double m1[2][2], double m2[2][2], double out[2][2])
{
	out[0][0] = m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0];
	out[0][1] = m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1];
	out[1][0] = m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0];
	out[1][1] = m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1];
}

//==================================================================================================================

inline void mat_mult3x4(const double m[3][4], const double v[4], double out[3])
{
	out[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2] + m[0][3]*v[3];
	out[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2] + m[1][3]*v[3];
	out[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2] + m[2][3]*v[3];
}

//==================================================================================================================

inline void mat_mult3_by_hom_mat(const double** mat4x4, const double* p3, double* out3)
{
	mat_mult3x3(mat4x4, p3, out3);
	out3[0] += mat4x4[0][3];
	out3[1] += mat4x4[1][3];
	out3[2] += mat4x4[2][3];
}

//==================================================================================================================

template<class T>
inline void mat_mult3_by_rigid(const T* rigid, const T* p, T* out)
{
	// Rotate and translate
	out[0] = rigid[0]*p[0] + rigid[1]*p[1] + rigid[2]*p[2] + rigid[9];
	out[1] = rigid[3]*p[0] + rigid[4]*p[1] + rigid[5]*p[2] + rigid[10];
	out[2] = rigid[6]*p[0] + rigid[7]*p[1] + rigid[8]*p[2] + rigid[11];
}

//==================================================================================================================

/** out = sR + t, where (R, t) = T and R is a 9-vector and t is a 3-vector, i.e., T is a 12-vector.
  * Note that R is a rotation matrix saved as a 9-vector. */
template<class T>
inline void mat_mult3_by_similarity(T s, const T* rigid, const T* p, T* out)
{
	// Rotate, scale and translate
	out[0] = s*(rigid[0]*p[0] + rigid[1]*p[1] + rigid[2]*p[2]) + rigid[9];
	out[1] = s*(rigid[3]*p[0] + rigid[4]*p[1] + rigid[5]*p[2]) + rigid[10];
	out[2] = s*(rigid[6]*p[0] + rigid[7]*p[1] + rigid[8]*p[2]) + rigid[11];
}

//==================================================================================================================

inline void mat_add_tensor_product_to_mat3x3(const double* a, const double* b, double mat[3][3])
{
	mat[0][0] += a[0]*b[0]; mat[0][1] += a[0]*b[1]; mat[0][2] += a[0]*b[2];
	mat[1][0] += a[1]*b[0]; mat[1][1] += a[1]*b[1]; mat[1][2] += a[1]*b[2];
	mat[2][0] += a[2]*b[0]; mat[2][1] += a[2]*b[1]; mat[2][2] += a[2]*b[2];
}

//==================================================================================================================

inline void mat_add_tensor_product_to_mat9(const double* a, const double* b, double* mat)
{
	mat[0] += a[0]*b[0]; mat[1] += a[0]*b[1]; mat[2] += a[0]*b[2];
	mat[3] += a[1]*b[0]; mat[4] += a[1]*b[1]; mat[5] += a[1]*b[2];
	mat[6] += a[2]*b[0]; mat[7] += a[2]*b[1]; mat[8] += a[2]*b[2];
}

//==================================================================================================================

inline double mat_determinant3x3(const double** m)
{
	return
	 m[0][0]*(m[1][1]*m[2][2] - m[2][1]*m[1][2])
	-m[1][0]*(m[0][1]*m[2][2] - m[2][1]*m[0][2])
	+m[2][0]*(m[0][1]*m[1][2] - m[1][1]*m[0][2]);
}

//==================================================================================================================

inline double mat_determinant3x3(const double m[3][3])
{
	return
	 m[0][0]*(m[1][1]*m[2][2] - m[2][1]*m[1][2])
	-m[1][0]*(m[0][1]*m[2][2] - m[2][1]*m[0][2])
	+m[2][0]*(m[0][1]*m[1][2] - m[1][1]*m[0][2]);
}

//==================================================================================================================

template<class T>
inline double mat_determinant9(const T* m)
{
	return
	 m[0]*(m[4]*m[8] - m[7]*m[5])
	-m[3]*(m[1]*m[8] - m[7]*m[2])
	+m[6]*(m[1]*m[5] - m[4]*m[2]);
}

//==================================================================================================================

inline double mat_trace4x4(const double** m)
{
	return m[0][0] + m[1][1] + m[2][2] + m[3][3];
}

//==================================================================================================================

inline double mat_trace3x3(const double** m)
{
	return m[0][0] + m[1][1] + m[2][2];
}

//==================================================================================================================

inline void mat_sum3x3(const double a[3][3], const double b[3][3], double out[3][3])
{
	out[0][0] = a[0][0]+b[0][0]; out[0][1] = a[0][1]+b[0][1]; out[0][2] = a[0][2]+b[0][2];
	out[1][0] = a[1][0]+b[1][0]; out[1][1] = a[1][1]+b[1][1]; out[1][2] = a[1][2]+b[1][2];
	out[2][0] = a[2][0]+b[2][0]; out[2][1] = a[2][1]+b[2][1]; out[2][2] = a[2][2]+b[2][2];
}

//==================================================================================================================

inline void mat_diff3x3(const double a[3][3], const double b[3][3], double out[3][3])
{
	out[0][0] = a[0][0]-b[0][0]; out[0][1] = a[0][1]-b[0][1]; out[0][2] = a[0][2]-b[0][2];
	out[1][0] = a[1][0]-b[1][0]; out[1][1] = a[1][1]-b[1][1]; out[1][2] = a[1][2]-b[1][2];
	out[2][0] = a[2][0]-b[2][0]; out[2][1] = a[2][1]-b[2][1]; out[2][2] = a[2][2]-b[2][2];
}

//==================================================================================================================

inline double mat_frobeniusDist3x3(const double A[3][3], const double B[3][3])
{
	return sqrt(
			M_SQR(A[0][0]-B[0][0]) + M_SQR(A[0][1]-B[0][1]) + M_SQR(A[0][2]-B[0][2]) +
			M_SQR(A[1][0]-B[1][0]) + M_SQR(A[1][1]-B[1][1]) + M_SQR(A[1][2]-B[1][2]) +
			M_SQR(A[2][0]-B[2][0]) + M_SQR(A[2][1]-B[2][1]) + M_SQR(A[2][2]-B[2][2]));
}

//==================================================================================================================

inline double mat_frobenius_dist9(const double A[9], const double B[9])
{
	return sqrt(
			M_SQR(A[0]-B[0]) + M_SQR(A[1]-B[1]) + M_SQR(A[2]-B[2]) +
			M_SQR(A[3]-B[3]) + M_SQR(A[4]-B[4]) + M_SQR(A[5]-B[5]) +
			M_SQR(A[6]-B[6]) + M_SQR(A[7]-B[7]) + M_SQR(A[8]-B[8]));
}

//==================================================================================================================

inline double mat_dist3x3L1(const double A[3][3], const double B[3][3])
{
	return
	fabs(A[0][0]-B[0][0]) + fabs(A[0][1]-B[0][1]) + fabs(A[0][2]-B[0][2]) +
	fabs(A[1][0]-B[1][0]) + fabs(A[1][1]-B[1][1]) + fabs(A[1][2]-B[1][2]) +
	fabs(A[2][0]-B[2][0]) + fabs(A[2][1]-B[2][1]) + fabs(A[2][2]-B[2][2]);
}

//==================================================================================================================

inline void mat_subtract4x4(const double** a, const double** b, double** out)
{
	out[0][0]=a[0][0]-b[0][0]; out[0][1]=a[0][1]-b[0][1]; out[0][2]=a[0][2]-b[0][2]; out[0][3]=a[0][3]-b[0][3];
	out[1][0]=a[1][0]-b[1][0]; out[1][1]=a[1][1]-b[1][1]; out[1][2]=a[1][2]-b[1][2]; out[1][3]=a[1][3]-b[1][3];
	out[2][0]=a[2][0]-b[2][0]; out[2][1]=a[2][1]-b[2][1]; out[2][2]=a[2][2]-b[2][2]; out[2][3]=a[2][3]-b[2][3];
	out[3][0]=a[3][0]-b[3][0]; out[3][1]=a[3][1]-b[3][1]; out[3][2]=a[3][2]-b[3][2]; out[3][3]=a[3][3]-b[3][3];
}

//==================================================================================================================

inline void mat_subtract3x3(const double** a, const double** b, double** out)
{
	out[0][0]=a[0][0]-b[0][0]; out[0][1]=a[0][1]-b[0][1]; out[0][2]=a[0][2]-b[0][2];
	out[1][0]=a[1][0]-b[1][0]; out[1][1]=a[1][1]-b[1][1]; out[1][2]=a[1][2]-b[1][2];
	out[2][0]=a[2][0]-b[2][0]; out[2][1]=a[2][1]-b[2][1]; out[2][2]=a[2][2]-b[2][2];
}

//==================================================================================================================

inline void mat_set_to_identity3x3(double m[3][3])
{
	m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0;
	m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0;
	m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0;
}

//==================================================================================================================

/** Swaps the columns 'i' and 'j' of the 3x3 matrix 'm'. */
inline void mat_swap_cols3x3(double m[3][3], int i, int j)
{
	double a, b, c;
	// [a,b,c] = col1
	a = m[0][i];
    b = m[1][i];
	c = m[2][i];
	// col1 = col2
	m[0][i] = m[0][j];
	m[1][i] = m[1][j];
	m[2][i] = m[2][j];
	//  col2 = [a,b,c]
	m[0][j] = a;
	m[1][j] = b;
	m[2][j] = c;
}

//==================================================================================================================

inline void mat_change_row_sign3x3(double M[3][3], int row)
{
	M[row][0] = -M[row][0];
	M[row][1] = -M[row][1];
	M[row][2] = -M[row][2];
}

//==================================================================================================================

inline void mat_change_col_sign3x3(double M[3][3], int col)
{
	M[0][col] = -M[0][col];
	M[1][col] = -M[1][col];
	M[2][col] = -M[2][col];
}

//==================================================================================================================

inline void mat_descending_sort_cols(double U[3][3], double w[3])
{
	// First run
	if ( w[2] > w[1] )
	{
		vec_swap(w, 2, 1);
		mat_swap_cols3x3(U, 2, 1);
	}
	if ( w[1] > w[0] )
	{
		vec_swap(w, 1, 0);
		mat_swap_cols3x3(U, 1, 0);
	}
	// Second run
	if ( w[2] > w[1] )
	{
		vec_swap(w, 2, 1);
		mat_swap_cols3x3(U, 2, 1);
	}
}

//==================================================================================================================

/** Computes a polar decomposition for 'M' and saves the rotation matrix in 'R'. */
inline void mat_svd_based_polar_decomposition3x3(const double M[3][3], double R[3][3])
{
	double w[3], MT[3][3], MMT[3][3], MTM[3][3], U[3][3], UT[3][3], V[3][3], VT[3][3];
	mat_transpose3x3(M, MT);
	mat_mult3x3(M, MT, MMT);
	mat_mult3x3(MT, M, MTM);

	btl1_dsyevh3(MMT, U, w); // Eigendecomposition
	mat_descending_sort_cols(U, w);
	btl1_dsyevh3(MTM, V, w); // Eigendecomposition
	mat_descending_sort_cols(V, w);
	mat_transpose3x3(U, UT);
	mat_transpose3x3(V, VT);

	// Compute S (the S involved in M = U*S*V^T)
	double MV[3][3];
	mat_mult3x3(M, V, MV);

	// S is a diagonal matrix => compute only the diagonal entries and check the sign of each one
	if ( UT[0][0]*MV[0][0] + UT[0][1]*MV[1][0] + UT[0][2]*MV[2][0] < 0.0 )
		mat_change_col_sign3x3(U, 0);
	if ( UT[1][0]*MV[0][1] + UT[1][1]*MV[1][1] + UT[1][2]*MV[2][1] < 0.0 )
		mat_change_col_sign3x3(U, 1);
	if ( UT[2][0]*MV[0][2] + UT[2][1]*MV[1][2] + UT[2][2]*MV[2][2] < 0.0 )
		mat_change_col_sign3x3(U, 2);

	mat_mult3x3(U, VT, R);

#if 0
	// begin {paranoia check}
	mat_mult3x3(U, S, MT);
	mat_mult3x3(MT, VT, MTM);
	printf("\n\nUSV^T = \n");
	mat_print3x3(MTM);
	printf("M = \n");
	mat_print3x3(M);
	// end {paranoia check}
#endif

	if ( mat_determinant3x3(R) < 0.0 )
	{
		VT[2][0] = -VT[2][0];
		VT[2][1] = -VT[2][1];
		VT[2][2] = -VT[2][2];
		mat_mult3x3(U, VT, R);
	}
}

//==================================================================================================================

#define MPD_MAX_INV  10000.0

inline void mat_polar_decomposition3x3(const double M[3][3], double R[3][3])
{
	// The following method is used to compute the rotation matrix R based
	// on the polar decomposition M = RS:
	// (we denote the transpose of A by A')
	// 1) S is symmetric positive semi-definite => it has an eigendecomposition S = UKU',
	//    where U is a rotation matrix with columns being the eigenvectors of S and K is a diagonal matrix
	//    consisting of the eigenvalues of S
	// 2) Compute MTM = M'M = S'S = UK'KU' (MTM is symmetric)
	// 3) Compute the eignedecomposition MTM = UDU'
	//    =>  K'K = D = diag(w_1, w_2, w_3)  =>  K = diag(sqrt(w_1), sqrt(w_2), sqrt(w_3))
	//    =>   K^(-1) = diag(1/sqrt(w_1), 1/sqrt(w_2), 1/sqrt(w_3))
	// 4) Compute Q = MUK^(-1)U'

	double w[3], MT[3][3], MTM[3][3], U[3][3];
	mat_transpose3x3(M, MT);
	mat_mult3x3(MT, M, MTM);
	// Compute the eigendecomposition of MTM
	btl1_dsyevh3(MTM, U, w);
	mat_descending_sort_cols(U, w);
	double KiUT[3][3]; // (K^-1)U^T

	// Compute U^T
	mat_transpose3x3(U, KiUT);
	// Compute (K^-1)U^T
	double ki0 = 1.0/sqrt(fabs(w[0])), ki1 = 1.0/sqrt(fabs(w[1])), ki2 = 1.0/sqrt(fabs(w[2]));

	if ( ki0 > MPD_MAX_INV || ki1  > MPD_MAX_INV || ki2  > MPD_MAX_INV )
		mat_svd_based_polar_decomposition3x3(M, R);
	else
	{
		mat_mult_rows3x3(KiUT, ki0, ki1, ki2);
		// Compute the rotation matrix
		double MU[3][3]; mat_mult3x3(M, U, MU);
		mat_mult3x3(MU, KiUT, R);

		if ( mat_determinant3x3(R) < 0.0 )
		{
			mat_change_row_sign3x3(KiUT, 2);
			mat_mult3x3(MU, KiUT, R);
		}
	}
}


}//namespace tum

#endif /* MATRIX_H_ */
