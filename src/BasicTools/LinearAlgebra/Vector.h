#ifndef _TUM_VECTOR_H_
#define _TUM_VECTOR_H_

#include <cstdio>
#include <cmath>

namespace tum
{

class Vector
{
public:
	inline Vector();
	inline Vector(int dimension);
	inline Vector(const Vector& src);
	inline virtual ~Vector();

	inline void multiply(double s);
	inline void divide(double s);
	/** Divides every element of this vector by @param s and saves the resulting vector
	  * in @param dst. This vector is not altered. */
	inline void divide(double s, Vector& dst);

	/** this = v1 - v2. */
	inline void difference(const Vector& v1, const Vector& v2);
	/** this = v1 + v2. */
	inline void sum(const Vector& v1, const Vector& v2);

	inline void operator +=(const Vector& v);
	inline void operator -=(const Vector& v);
	inline void operator  =(const Vector& src);

	/** This method assumes that this vector and 'src' have the same dimension! */
	inline void copyto(double* dst);
	/** This method assumes that this vector and 'dst' have the same dimension! */
	inline void copyfrom(const double* src);
	/** This method assumes that this vector and 'src' have the same dimension! */
	inline void copyFrom(const Vector& src);
	inline void copyTo(double* dst);

	/** out = a - b. */
	static inline void diff(const double a[3], const double b[3], double out[3]);

	/** Normalize the vector 'v'. */
	static inline void normalize3(double v[3]);
	/** Save the normalized 'v' in out. */
	static inline void normalize3(const double* v, double* out);
	/** v = v*s. */
	static inline void mult3(double *v, double s){ v[0] *= s; v[1] *= s; v[2] *= s;}
	/** out = v*s */
	static inline void mult3(const double *v, double s, double* out){ out[0] = s*v[0]; out[1] = s*v[1]; out[2] = s*v[2];}
	static inline void sum3(const double *a, const double *b, double *c){ c[0] = a[0]+b[0]; c[1] = a[1]+b[1]; c[2] = a[2]+b[2];}
	static inline void set3(double *vec, double scalar){ vec[0] = vec[1] = vec[2] = scalar;}
	/** v = -v. */
	static inline void flip3(double *v){ v[0] = -v[0]; v[1] = -v[1]; v[2] = -v[2];}
	/** a += v. */
	static inline void add3(double *a, const double *v){ a[0] += v[0]; a[1] += v[1]; a[2] += v[2];}
	/** a += (x, y, z). */
	static inline void add3(double* a, double x, double y, double z){ a[0] += x; a[1] += y; a[2] += z;}
	/** Cross product. */
	static inline void cross3(const double v1[3], const double v2[3], double out[3]);
	/** Dot product. */
	static inline double dot3(const double v1[3], const double v2[3]);
	static inline double dist3(const double a[3], const double b[3]);
	static inline double dist3L1(const double* a, const double* b);
	static inline double sqrDist3(const double a[3], const double b[3]);
	static inline double length3(const double *v);
	static inline double sqrLength3(const double *v);
	/** Returns the distance between the point 'x' and the line defined by the
	  * vector 'unitLineVec' and the point 'linePoint'. */
	static inline double dist3(const double x[3], const double unitLineVec[3], const double linePoint[3]);
	static inline void copy3(const double src[3], double dst[3]);
	/** Copies the 'n'-dimensional 'src' to 'dst'. */
	static inline void copy(const double* src, double* dst, int n);
	/** Returns the signed distance from 'point' to the plane defined by 'planePoint' and 'planeNormal'. */
	static inline double signedDistToPlane3(const double* planePoint, const double* planeNormal, const double* point);

	inline void print(FILE* dst);
	static inline void print3(double* v, FILE* stream, char *label);

protected:
	inline void alloc(int dimension);
	inline void dealloc();

public:
	double *mData;
	int mDimension;
};

}//namespace tum

//=== inline methods =================================================================================================

inline tum::Vector::Vector()
{
	mDimension = 0;
	mData = NULL;
}

//====================================================================================================================

inline tum::Vector::Vector(int dimension)
{
	this->alloc(dimension);
}

//====================================================================================================================

inline tum::Vector::Vector(const Vector& src)
{
	this->alloc(src.mDimension);
	this->copyFrom(src);
}

//====================================================================================================================

inline tum::Vector::~Vector()
{
	this->dealloc();
}

//====================================================================================================================

inline void tum::Vector::alloc(int dimension)
{
	mDimension = dimension;
	mData = new double[dimension];
}

//====================================================================================================================

inline void tum::Vector::dealloc()
{
	if ( mData == NULL )
		return;

	delete[] mData;
	mData = NULL;
	mDimension = 0;
}

//====================================================================================================================

void tum::Vector::print(FILE* dst)
{
	fprintf(dst, "(");
	for ( int i = 0 ; i < mDimension-1 ; ++i )
		fprintf(dst, "%lf, ", mData[i]);
	fprintf(dst, "%lf)\n", mData[mDimension-1]);
}

//====================================================================================================================

void tum::Vector::print3(double* v, FILE* stream, char *label)
{
	if ( label )
		fprintf(stream, "%s = (%lf, %lf, %lf)\n", label, v[0], v[1], v[2]);
	else
		fprintf(stream, "v = (%lf, %lf, %lf)\n", v[0], v[1], v[2]);

	fflush(stream);
}

//====================================================================================================================

inline void tum::Vector::multiply(double s)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		mData[i] *= s;
}

//====================================================================================================================

inline void tum::Vector::divide(double s)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		mData[i] /= s;
}

//====================================================================================================================

inline void tum::Vector::divide(double s, Vector& dst)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		dst.mData[i] = mData[i]/s;
}

//====================================================================================================================

inline void tum::Vector::operator +=(const Vector& v)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		mData[i] += v.mData[i];
}

//====================================================================================================================

inline void tum::Vector::operator -=(const Vector& v)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		mData[i] -= v.mData[i];
}

//====================================================================================================================

inline void tum::Vector::operator =(const Vector& src)
{
	this->dealloc();
	this->alloc(src.mDimension);
	this->copyFrom(src);
}

//====================================================================================================================

inline void tum::Vector::difference(const Vector& v1, const Vector& v2)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		mData[i] = v1.mData[i] - v2.mData[i];
}

//====================================================================================================================

inline void tum::Vector::sum(const Vector& v1, const Vector& v2)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		mData[i] = v1.mData[i] + v2.mData[i];
}

//====================================================================================================================

inline void tum::Vector::copyFrom(const Vector& src)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		mData[i] = src.mData[i];
}

//====================================================================================================================

inline void tum::Vector::copyto(double* dst)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		dst[i] = mData[i];
}

//====================================================================================================================

inline void tum::Vector::copyfrom(const double* src)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		mData[i] = src[i];
}

//====================================================================================================================

inline void tum::Vector::copyTo(double* dst)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		dst[i] = mData[i];
}

//====================================================================================================================

inline void tum::Vector::normalize3(double v[3])
{
	double inv_len = 1.0/sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] *= inv_len;
	v[1] *= inv_len;
	v[2] *= inv_len;
}

//====================================================================================================================

inline void tum::Vector::normalize3(const double* v, double* out)
{
	double inv_len = 1.0/sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	out[0] = inv_len*v[0];
	out[1] = inv_len*v[1];
	out[2] = inv_len*v[2];
}

//====================================================================================================================

inline void tum::Vector::cross3(const double v1[3], const double v2[3], double out[3])
{
	out[0] = v1[1]*v2[2] - v1[2]*v2[1];
	out[1] = v1[2]*v2[0] - v1[0]*v2[2];
	out[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

//====================================================================================================================

inline double tum::Vector::dot3(const double v1[3], const double v2[3])
{
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

//====================================================================================================================

inline double tum::Vector::dist3(const double a[3], const double b[3])
{
	return sqrt(pow(a[0]-b[0], 2.0) + pow(a[1]-b[1], 2.0) + pow(a[2]-b[2], 2.0));
}

//====================================================================================================================

inline double tum::Vector::dist3L1(const double* a, const double* b)
{
	return fabs(a[0]-b[0]) + fabs(a[1]-b[1]) + fabs(a[2]-b[2]);
}

//====================================================================================================================

inline double tum::Vector::sqrDist3(const double a[3], const double b[3])
{
	return pow(a[0]-b[0],2) + pow(a[1]-b[1],2) + pow(a[2]-b[2],2);
}

//====================================================================================================================

inline double tum::Vector::dist3(const double x[3], const double unitLineVec[3], const double linePoint[3])
{
	double y[3] = {x[0]-linePoint[0], x[1]-linePoint[1], x[2]-linePoint[2]};
	double dy = unitLineVec[0]*y[0] + unitLineVec[1]*y[1] + unitLineVec[2]*y[2];

	return sqrt(fabs(y[0]*y[0] + y[1]*y[1] + y[2]*y[2] - dy*dy));
}

//====================================================================================================================

inline void tum::Vector::copy3(const double src[3], double dst[3])
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
}

//====================================================================================================================

inline void tum::Vector::copy(const double* src, double* dst, int n)
{
	for ( int i = 0 ; i < n ; ++i )
		dst[i] = src[i];
}

//====================================================================================================================

inline double tum::Vector::signedDistToPlane3(const double* planePoint, const double* planeNormal, const double* point)
{
	double v[3] = {point[0]-planePoint[0], point[1]-planePoint[1], point[2]-planePoint[2]};
	return tum::Vector::dot3(v, planeNormal);
}

//====================================================================================================================

inline void tum::Vector::diff(const double a[3], const double b[3], double out[3])
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

//====================================================================================================================

inline double tum::Vector::length3(const double *v)
{
	return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

//====================================================================================================================

inline double tum::Vector::sqrLength3(const double *v)
{
	return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}

//====================================================================================================================

#endif /*_TUM_VECTOR_H_*/
