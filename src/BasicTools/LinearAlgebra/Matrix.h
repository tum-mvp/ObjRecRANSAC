#ifndef _TUM_MATRIX_H_
#define _TUM_MATRIX_H_

#include <vtkMath.h>
#include <cstdio>
#include <cmath>

namespace tum
{

/** Very simple matrix operations like multiplication, transposition, ...
 * */
class Matrix
{
public:
	Matrix();
	Matrix(int rows, int cols);
	virtual ~Matrix();

	/** Allocates memory for a matrix with rows x cols double elements. */
	void alloc(int rows, int cols);
	/** Frees the memory for this matrix. */
	void dealloc();
	/** Sets this matrix to identity. */
	void setToIdentity();
	/** Writes the elements of this matrix to file. If label != NULL, then the string label will be printed too. */
	void print(FILE* file, const char* label = NULL);
	void printAsInt(FILE* file, const char* label = NULL);

	/** Returns the number of columns of this matrix. */
	int getNumberOfColumns(){ return mCols;}
	/** Returns the number of rows of this matrix. */
	int getNumberOfRows(){ return mRows;}

	/** Returns the matrix elements. */
	const double** getm(){ return (const double**)m;}

	/** Multiplies every element of this matrix with 'value'. */
	inline void mult(double value);

	/** out = mat*u. 'mat' is a matrix and 'u' and 'out' are vectors. Of course, the matrix and the
	 * vectors have to be compatible. */
	inline static void mult(const double** mat, int rows, int cols, const double* u, double *out);

	/** out = m1*m2. */
	inline static void mult(const Matrix* m1, const Matrix* m2, Matrix* out);
	/** Transposes src and saves the result in dst.  */
	inline static void transpose3x3(const double src[3][3], double dst[3][3]);
	/** Transposes src and saves the result in dst.  */
	inline static void transpose3x3(const double** src, double** dst);
	/** Transposes 'src' and saves the result in 'dst'.  */
	inline static void transpose4x4(const double** src, double** dst);
	/** Sets mat to the 3x3 identity matrix. */
	inline static void setToIdentity3x3(double mat[3][3]);

	/** Sets all elements of the 3x3 matrix to 's'. */
	inline static void set3x3(double m[3][3], double s);
	/** res += mat. */
	inline static void add3x3(double res[3][3], const double m[3][3]);
	/** res -= mat. */
	inline static void sub3x3(double res[3][3], const double m[3][3]);
	/** out = m1 * m2. */
	inline static void mult2x2(double **m1, double **m2, double **out);
	/** out = m1 * m2. */
	inline static void mult2x2(double m1[2][2], double m2[2][2], double out[2][2]);
	/** out = mat * v. */
	inline static void mult3x3(const double mat[3][3], const double v[3], double out[3]);
	/** out = mat * v. */
	inline static void mult3x3(const double mat[3][3], double x, double y, double z, double out[3]);
	/** out = mat * v. */
	inline static void mult3x3(const double** mat, const double* v, double* out);
	/** out = m1 * m2. */
	inline static void mult3x3(const double m1[3][3], const double m2[3][3], double out[3][3]);
	/** out = m1 * m2. */
	inline static void mult3x3(const double** m1, const double** m2, double** out);
	/** out = [x y z]*m, i.e., x, y and z are interpreted as the columns of the 3x3 matrix [x y z]. */
	inline static void mult3x3(const double* x, const double* y, const double* z, const double** m, double** out);
	/** out = ab^T. */
	inline static void tensor_product3x3(const double* a, const double* b, double out[3][3]);
	/** out = mat * v. */
	inline static void mult3x4(const double mat[3][4], const double v[4], double out[3]);
	/** This method multiplies the matrix m1 with the matrix m2. The result is saved in m2,
	  * i.e., m2 = m1 * m2. m2 has to be a matrix with 3 rows and numOfCols columns. */
	inline static void mult3xN(const double m1[3][3], double** m2, int numOfCols);
	/** out = mat * v. mat has to be a 4x4 matrix and v a 4d vector. */
	inline static void mult4x4(const double** mat, const double* v, double* out);
	/** out = m1*m2. All matrices have to be 4x4. */
	inline static void mult4x4(const double** m1, const double** m2, double** out);
	/** out3 = mat4x4 * p3, i.e. multiplication of the upper left block matrix of 'mat4x4' by 'p3'
	  * and then addition of the last column of 'mat4x4' to 'p3'. */
	inline static void mult3ByHomMat(const double** mat4x4, const double* p3, double* out3);
	/** 'rigid' has to be a 12D array: the first 9 element are from a rotation matrix (one row
	  * after another) and the last 3 elements are the translation. 'p3' is the point which will
	  * be rigidly transformed and 'out3' is the transformed point. */
	inline static void mult3ByRigid(const double* rigid, const double* p3, double* out3);
	/** Copies the matrix [x y z] to dst. x, y and z are interpreted as the columns of a 3x3 matrix. */
	inline static void copy3x3(const double* x, const double* y, const double* z, double** dst);
	/** Copies src to dst. */
	inline static void copy3x3(const double src[3][3], double dst[3][3]);
	/** Copies src to dst. */
	inline static void copy3x3(const double** src, double dst[3][3]);
	/** Copies src to dst. */
	inline static void copy3x3(double src[3][3], double** dst);
	/** Copies src to dst. */
	inline static void copy3x3(const double** src, double** dst);
	/** Copies src to dst. */
	inline static void copy3x3(const double src[3][3], double** dst);
	/** Copies the 3x3 matrix 'src' to the 9D vector 'dst'. It copies one row after another. */
	inline static void copy3x3(const double** src, float* dst);
	/** Copies the 3x3 matrix 'src' to the 9D vector 'dst'. It copies one row after another. */
	inline static void copy3x3(const double** src, double* dst);
	/** Copies src to dst. */
	inline static void copy4x4(double** src, double** dst);
	/** Copies the 12D vector 'src' to the 4x4 homogeneous matrix 'dst', i.e., the first
	  * 9 elements should be from a rotation matrix (one row after another) and the last
	  * 3 elements are the translation. */
	inline static void copyRigidTransformTo4x4(const double* src, double** hommat4x4);
	/** Computes the determinant of the 3x3 matrix 'm'. */
	inline static double determinant3x3(const double** m);
	/** Returns the trace of the 4x4 matrix 'm' */
	inline static double trace4x4(const double** m);
	/** Returns the trace of the 3x3 matrix 'm' */
	inline static double trace3x3(const double** m);
	/** out = a - b. */
	inline static void diff3x3(const double a[3][3], const double b[3][3], double out[3][3]);

	/** Returns the Frobenius norm of the 3x3 matrix 'm'. */
	inline static double frobeniusNorm3x3(const double** m);
	/** Returns the L1 norm of A - B. */
	inline static double dist3x3L1(const double A[3][3], const double B[3][3]);

	/** Computes the polar decomposition A = RU, where R is a rotation matrix and U is a "stretch"
	  * matrix. The rotation matrix is saved in 'R' and the stretch matrix U is not computed by
	  * this version of the method. */
	inline static void polarDecomposition3x3(const double A[3][3], double R[3][3]);

	/** 'out' = 'a' + 'b'. */
	inline static void add(const Matrix& a, const Matrix& b, Matrix& out);
	/** 'out' = 'a' - 'b'. */
	inline static void subtract4x4(const double** a, const double** b, double** out);
	/** 'out' = 'a' - 'b'. */
	inline static void subtract3x3(const double** a, const double** b, double** out);

	/** Copies this matrix to 'dst'. 'dst' has to have the same number of rows and columns as this matrix. */
	inline void copyto(double** dst)const;
	/** Copies this matrix to 'dst'. 'dst' has to have the same number of rows and columns as this matrix. */
	inline void copyto(Matrix* dst)const;
	/** Copies the elements from 'src' to this matrix. 'src' has to have the same number of rows and columns as this matrix. */
	inline void copyfrom(const double** src);
	/** mat += ab^T */
	inline static void add_tensor_product_to_mat3x3(const double* a, const double* b, double mat[3][3]);

	static bool loadFromFile4x4(double** mat4x4, const char* filename);
	static bool writeToFile4x4(const double** mat4x4, const char* filename);

	/** Rotates p by rad (in radiants) about the line defined by linePoint and lineAxis. lineAxis has to be
	  * normalized, i.e., have the length 1. */
	static void rotateAboutLine3(const double linePoint[3], const double lineAxis[3], double rad, double p[3]);
	/** Writes the elements of mat to file. If label != NULL, then the string label will be printed too. */
	static void print3x3(FILE* file, double mat[3][3], char* label = NULL);
	/** Writes the elements of 'mat' to file. If label != NULL, then the string label will be printed too. */
	static void print(FILE* file, const double** mat, int width, int height, char* label = NULL);

public:
	/** This pointer points to the elements of this matrix. */
	double **m;

protected:
	int mRows, mCols;
};

}//namespace tum

//=== inline methods ===============================================================================================

inline void tum::Matrix::mult(double value)
{
	int i, j;
	for ( i = 0 ; i < mRows ; ++i )
		for ( j = 0 ; j < mCols ; ++j )
			m[i][j] *= value;
}

//==================================================================================================================

inline void tum::Matrix::mult(const double** mat, int rows, int cols, const double* u, double *out)
{
	for ( int i = 0 ; i < rows ; ++i )
	{
		out[i] = 0.0;
		for ( int j = 0 ; j < cols ; ++j )
			out[i] += mat[i][j]*u[j];
	}
}

//==================================================================================================================

inline void tum::Matrix::copy3x3(const double** src, double dst[3][3])
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void tum::Matrix::copy3x3(double src[3][3], double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void tum::Matrix::copy3x3(const double src[3][3], double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void tum::Matrix::copy3x3(const double** src, double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void tum::Matrix::copy3x3(const double** src, float* dst)
{
	dst[0] = (float)src[0][0]; dst[1] = (float)src[0][1]; dst[2] = (float)src[0][2];
	dst[3] = (float)src[1][0]; dst[4] = (float)src[1][1]; dst[5] = (float)src[1][2];
	dst[6] = (float)src[2][0]; dst[7] = (float)src[2][1]; dst[8] = (float)src[2][2];
}

//==================================================================================================================

inline void tum::Matrix::copy3x3(const double** src, double* dst)
{
	dst[0] = src[0][0]; dst[1] = src[0][1]; dst[2] = src[0][2];
	dst[3] = src[1][0]; dst[4] = src[1][1]; dst[5] = src[1][2];
	dst[6] = src[2][0]; dst[7] = src[2][1]; dst[8] = src[2][2];
}

//==================================================================================================================

inline void tum::Matrix::copy4x4(double** src, double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2]; dst[0][3] = src[0][3];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2]; dst[1][3] = src[1][3];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2]; dst[2][3] = src[2][3];
	dst[3][0] = src[3][0]; dst[3][1] = src[3][1]; dst[3][2] = src[3][2]; dst[3][3] = src[3][3];
}

//==================================================================================================================

inline void tum::Matrix::copyRigidTransformTo4x4(const double* src, double** hommat4x4)
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

inline void tum::Matrix::mult3xN(const double m1[3][3], double** m2, int numOfCols)
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

inline void tum::Matrix::mult4x4(const double** mat, const double* v, double* out)
{
	out[0] = mat[0][0]*v[0] + mat[0][1]*v[1] + mat[0][2]*v[2] + mat[0][3]*v[3];
	out[1] = mat[1][0]*v[0] + mat[1][1]*v[1] + mat[1][2]*v[2] + mat[1][3]*v[3];
	out[2] = mat[2][0]*v[0] + mat[2][1]*v[1] + mat[2][2]*v[2] + mat[2][3]*v[3];
	out[3] = mat[3][0]*v[0] + mat[3][1]*v[1] + mat[3][2]*v[2] + mat[3][3]*v[3];
}

//==================================================================================================================

inline void tum::Matrix::setToIdentity()
{
	int i, j;
	for ( i = 0 ; i < mRows ; ++i )
		m[i][i] = 1.0;

	for ( i = 0 ; i < mRows ; ++i )
		for ( j = i+1 ; j < mCols ; ++j )
			m[i][j] = m[j][i] = 0.0;
}

//==================================================================================================================

inline void tum::Matrix::copy3x3(const double* x, const double* y, const double* z, double** dst)
{
	dst[0][0] = x[0]; dst[0][1] = y[0]; dst[0][2] = z[0];
	dst[1][0] = x[1]; dst[1][1] = y[1]; dst[1][2] = z[1];
	dst[2][0] = x[2]; dst[2][1] = y[2]; dst[2][2] = z[2];
}

//==================================================================================================================

inline void tum::Matrix::copy3x3(const double src[3][3], double dst[3][3])
{
	dst[0][0] = src[0][0]; dst[0][1] = src[0][1]; dst[0][2] = src[0][2];
	dst[1][0] = src[1][0]; dst[1][1] = src[1][1]; dst[1][2] = src[1][2];
	dst[2][0] = src[2][0]; dst[2][1] = src[2][1]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void tum::Matrix::transpose3x3(const double src[3][3], double dst[3][3])
{
	dst[0][0] = src[0][0]; dst[0][1] = src[1][0]; dst[0][2] = src[2][0];
	dst[1][0] = src[0][1]; dst[1][1] = src[1][1]; dst[1][2] = src[2][1];
	dst[2][0] = src[0][2]; dst[2][1] = src[1][2]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void tum::Matrix::transpose3x3(const double** src, double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[1][0]; dst[0][2] = src[2][0];
	dst[1][0] = src[0][1]; dst[1][1] = src[1][1]; dst[1][2] = src[2][1];
	dst[2][0] = src[0][2]; dst[2][1] = src[1][2]; dst[2][2] = src[2][2];
}

//==================================================================================================================

inline void tum::Matrix::transpose4x4(const double** src, double** dst)
{
	dst[0][0] = src[0][0]; dst[0][1] = src[1][0]; dst[0][2] = src[2][0]; dst[0][3] = src[3][0];
	dst[1][0] = src[0][1]; dst[1][1] = src[1][1]; dst[1][2] = src[2][1]; dst[1][3] = src[3][1];
	dst[2][0] = src[0][2]; dst[2][1] = src[1][2]; dst[2][2] = src[2][2]; dst[2][3] = src[3][2];
	dst[3][0] = src[0][3]; dst[3][1] = src[1][3]; dst[3][2] = src[2][3]; dst[3][3] = src[3][3];
}

//==================================================================================================================

inline void tum::Matrix::mult3x3(const double mat[3][3], const double v[3], double out[3])
{
	out[0] = v[0]*mat[0][0] + v[1]*mat[0][1] + v[2]*mat[0][2];
	out[1] = v[0]*mat[1][0] + v[1]*mat[1][1] + v[2]*mat[1][2];
	out[2] = v[0]*mat[2][0] + v[1]*mat[2][1] + v[2]*mat[2][2];
}

//==================================================================================================================

inline void tum::Matrix::mult3x3(const double mat[3][3], double x, double y, double z, double out[3])
{
	out[0] = x*mat[0][0] + y*mat[0][1] + z*mat[0][2];
	out[1] = x*mat[1][0] + y*mat[1][1] + z*mat[1][2];
	out[2] = x*mat[2][0] + y*mat[2][1] + z*mat[2][2];
}

//==================================================================================================================

inline void tum::Matrix::mult3x3(const double** mat, const double* v, double* out)
{
	out[0] = v[0]*mat[0][0] + v[1]*mat[0][1] + v[2]*mat[0][2];
	out[1] = v[0]*mat[1][0] + v[1]*mat[1][1] + v[2]*mat[1][2];
	out[2] = v[0]*mat[2][0] + v[1]*mat[2][1] + v[2]*mat[2][2];
}

//==================================================================================================================

inline void tum::Matrix::mult3x3(const double m1[3][3], const double m2[3][3], double out[3][3])
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

inline void tum::Matrix::mult3x3(const double** m1, const double** m2, double** out)
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

inline void tum::Matrix::mult3x3(const double* x, const double* y, const double* z, const double** m, double** out)
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

inline void tum::Matrix::tensor_product3x3(const double* a, const double* b, double out[3][3])
{
	out[0][0] = a[0]*b[0]; out[0][1] = a[0]*b[1]; out[0][2] = a[0]*b[2];
	out[1][0] = a[1]*b[0]; out[1][1] = a[1]*b[1]; out[1][2] = a[1]*b[2];
	out[2][0] = a[2]*b[0]; out[2][1] = a[2]*b[1]; out[2][2] = a[2]*b[2];
}

//==================================================================================================================

inline void tum::Matrix::mult4x4(const double** m1, const double** m2, double** out)
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

inline void tum::Matrix::set3x3(double m[3][3], double s)
{
	m[0][0] = m[0][1] = m[0][2] =
	m[1][0] = m[1][1] = m[1][2] =
	m[2][0] = m[2][1] = m[2][2] = s;
}

//==================================================================================================================

inline void tum::Matrix::add3x3(double res[3][3], const double m[3][3])
{
	res[0][0] += m[0][0]; res[0][1] += m[0][1]; res[0][2] += m[0][2];
	res[1][0] += m[1][0]; res[1][1] += m[1][1]; res[1][2] += m[1][2];
	res[2][0] += m[2][0]; res[2][1] += m[2][1]; res[2][2] += m[2][2];
}

//==================================================================================================================

inline void tum::Matrix::sub3x3(double res[3][3], const double m[3][3])
{
	res[0][0] -= m[0][0]; res[0][1] -= m[0][1]; res[0][2] -= m[0][2];
	res[1][0] -= m[1][0]; res[1][1] -= m[1][1]; res[1][2] -= m[1][2];
	res[2][0] -= m[2][0]; res[2][1] -= m[2][1]; res[2][2] -= m[2][2];
}

//==================================================================================================================

inline void tum::Matrix::mult2x2(double **m1, double **m2, double **out)
{
	out[0][0] = m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0];
	out[0][1] = m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1];
	out[1][0] = m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0];
	out[1][1] = m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1];
}

//==================================================================================================================

inline void tum::Matrix::mult2x2(double m1[2][2], double m2[2][2], double out[2][2])
{
	out[0][0] = m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0];
	out[0][1] = m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1];
	out[1][0] = m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0];
	out[1][1] = m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1];
}

//==================================================================================================================

inline void tum::Matrix::mult3x4(const double m[3][4], const double v[4], double out[3])
{
	out[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2] + m[0][3]*v[3];
	out[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2] + m[1][3]*v[3];
	out[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2] + m[2][3]*v[3];
}

//==================================================================================================================

inline void tum::Matrix::mult3ByHomMat(const double** mat4x4, const double* p3, double* out3)
{
	tum::Matrix::mult3x3(mat4x4, p3, out3);
	out3[0] += mat4x4[0][3];
	out3[1] += mat4x4[1][3];
	out3[2] += mat4x4[2][3];
}

//==================================================================================================================

inline void tum::Matrix::mult3ByRigid(const double* rigid, const double* p, double* out)
{
	// Rotate
	out[0] = rigid[0]*p[0] + rigid[1]*p[1] + rigid[2]*p[2];
	out[1] = rigid[3]*p[0] + rigid[4]*p[1] + rigid[5]*p[2];
	out[2] = rigid[6]*p[0] + rigid[7]*p[1] + rigid[8]*p[2];
	// Translate
	out[0] += rigid[9];
	out[1] += rigid[10];
	out[2] += rigid[11];
}

//==================================================================================================================

inline void tum::Matrix::mult(const Matrix* m1, const Matrix* m2, Matrix* out)
{
	int i, j, k;
	for ( i = 0 ; i < m1->mRows ; ++i )
	{
		for ( j = 0 ; j < m2->mCols ; ++j )
		{
			out->m[i][j] = 0.0;
			for ( k = 0 ; k < m2->mRows ; ++k )
				out->m[i][j] += m1->m[i][k]*m2->m[k][j];
		}
	}
}

//==================================================================================================================

inline void tum::Matrix::copyto(double** dst) const
{
	int i, j;
	for ( i = 0 ; i < mRows ; ++i )
		for ( j = 0 ; j < mCols ; ++j )
			dst[i][j] = m[i][j];
}

//==================================================================================================================

inline void tum::Matrix::copyto(Matrix* dst) const
{
	int i, j;
	for ( i = 0 ; i < mRows ; ++i )
		for ( j = 0 ; j < mCols ; ++j )
			dst->m[i][j] = m[i][j];
}

//==================================================================================================================

inline void tum::Matrix::copyfrom(const double** src)
{
	int i, j;
	for ( i = 0 ; i < mRows ; ++i )
		for ( j = 0 ; j < mCols ; ++j )
			m[i][j] = src[i][j];
}

//==================================================================================================================

inline void tum::Matrix::add_tensor_product_to_mat3x3(const double* a, const double* b, double mat[3][3])
{
	mat[0][0] += a[0]*b[0]; mat[0][1] += a[0]*b[1]; mat[0][2] += a[0]*b[2];
	mat[1][0] += a[1]*b[0]; mat[1][1] += a[1]*b[1]; mat[1][2] += a[1]*b[2];
	mat[2][0] += a[2]*b[0]; mat[2][1] += a[2]*b[1]; mat[2][2] += a[2]*b[2];
}

//==================================================================================================================

inline double tum::Matrix::determinant3x3(const double** m)
{
	return
	 m[0][0]*(m[1][1]*m[2][2] - m[2][1]*m[1][2])
	-m[1][0]*(m[0][1]*m[2][2] - m[2][1]*m[0][2])
	+m[2][0]*(m[0][1]*m[1][2] - m[1][1]*m[0][2]);
}

//==================================================================================================================

inline double tum::Matrix::trace4x4(const double** m)
{
	return m[0][0] + m[1][1] + m[2][2] + m[3][3];
}

//==================================================================================================================

inline double tum::Matrix::trace3x3(const double** m)
{
	return m[0][0] + m[1][1] + m[2][2];
}

//==================================================================================================================

inline void tum::Matrix::diff3x3(const double a[3][3], const double b[3][3], double out[3][3])
{
	out[0][0] = a[0][0]-b[0][0]; out[0][1] = a[0][1]-b[0][1]; out[0][2] = a[0][2]-b[0][2];
	out[1][0] = a[1][0]-b[1][0]; out[1][1] = a[1][1]-b[1][1]; out[1][2] = a[1][2]-b[1][2];
	out[2][0] = a[2][0]-b[2][0]; out[2][1] = a[2][1]-b[2][1]; out[2][2] = a[2][2]-b[2][2];
}

//==================================================================================================================

inline double tum::Matrix::frobeniusNorm3x3(const double** m)
{
	Matrix mT(3, 3), prod(3, 3);
	Matrix::transpose3x3(m, mT.m);
	Matrix::mult3x3((const double**)mT.m, m, prod.m);

	return sqrt(Matrix::trace3x3((const double**)prod.m));
}

//==================================================================================================================

inline double tum::Matrix::dist3x3L1(const double A[3][3], const double B[3][3])
{
	return
	fabs(A[0][0]-B[0][0]) + fabs(A[0][1]-B[0][1]) + fabs(A[0][2]-B[0][2]) +
	fabs(A[1][0]-B[1][0]) + fabs(A[1][1]-B[1][1]) + fabs(A[1][2]-B[1][2]) +
	fabs(A[2][0]-B[2][0]) + fabs(A[2][1]-B[2][1]) + fabs(A[2][2]-B[2][2]);
}

//==================================================================================================================

inline void tum::Matrix::polarDecomposition3x3(const double A[3][3], double R[3][3])
{
	double U[3][3], w[3], VT[3][3];
	vtkMath::SingularValueDecomposition3x3(A, U, w, VT);
	vtkMath::Multiply3x3(U, VT, R);
}

//==================================================================================================================

inline void tum::Matrix::add(const Matrix& a, const Matrix& b, Matrix& c)
{
	int i, j;
	for ( i = 0 ; i < a.mRows ; ++i )
		for ( j = 0 ; j < a.mCols ; ++j )
			c.m[i][j] = a.m[i][j] + b.m[i][j];
}

//==================================================================================================================

inline void tum::Matrix::subtract4x4(const double** a, const double** b, double** out)
{
	out[0][0]=a[0][0]-b[0][0]; out[0][1]=a[0][1]-b[0][1]; out[0][2]=a[0][2]-b[0][2]; out[0][3]=a[0][3]-b[0][3];
	out[1][0]=a[1][0]-b[1][0]; out[1][1]=a[1][1]-b[1][1]; out[1][2]=a[1][2]-b[1][2]; out[1][3]=a[1][3]-b[1][3];
	out[2][0]=a[2][0]-b[2][0]; out[2][1]=a[2][1]-b[2][1]; out[2][2]=a[2][2]-b[2][2]; out[2][3]=a[2][3]-b[2][3];
	out[3][0]=a[3][0]-b[3][0]; out[3][1]=a[3][1]-b[3][1]; out[3][2]=a[3][2]-b[3][2]; out[3][3]=a[3][3]-b[3][3];
}

//==================================================================================================================

inline void tum::Matrix::subtract3x3(const double** a, const double** b, double** out)
{
	out[0][0]=a[0][0]-b[0][0]; out[0][1]=a[0][1]-b[0][1]; out[0][2]=a[0][2]-b[0][2];
	out[1][0]=a[1][0]-b[1][0]; out[1][1]=a[1][1]-b[1][1]; out[1][2]=a[1][2]-b[1][2];
	out[2][0]=a[2][0]-b[2][0]; out[2][1]=a[2][1]-b[2][1]; out[2][2]=a[2][2]-b[2][2];
}

//==================================================================================================================

inline void tum::Matrix::setToIdentity3x3(double m[3][3])
{
	m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0;
	m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0;
	m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0;
}

//==================================================================================================================

#endif /*_TUM_MATRIX_H_*/
