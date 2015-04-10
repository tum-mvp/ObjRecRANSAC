/*
 * ObjRecICP.h
 *
 *  Created on: Mar 23, 2011
 *      Author: papazov
 */

#ifndef _OBJ_REC_ICP_H_
#define _OBJ_REC_ICP_H_

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointLocator.h>
#include <opencv/cxcore.h>
#include <climits>
#include <ObjRecRANSAC/ObjRecRANSAC.h>
#include <ObjRecRANSAC/Shapes/PointSetShape.h>

#include <boost/shared_ptr.hpp>

//#define OBJ_REC_ICP_PRINT_ENERGY_LEVEL

class ObjRecICP
{
public:
	ObjRecICP();
	virtual ~ObjRecICP();

	void doICP(ObjRecRANSAC& objrec, list<boost::shared_ptr<PointSetShape> >& detectedShapes);

	inline void setEpsilon(double value){ mEpsilon = value;}
	inline void setTarget(vtkPolyData* target);

protected:
	/** Register the 'source' to the target (which was set with the 'setTarget()' method). The transformed points will be
	  * saved in 'out' and the rigid transformation will be saved in 'mat'. The method returns the RMS error. */
	double doRegistration(vtkPoints* source, vtkPoints* out, double mat[4][4], int maxNumOfIterations = INT_MAX,
			const double **init_mat = NULL);

	inline void cvPolarDecomp(const double M[3][3], double R[3][3]);

	inline void add3(double *v, const double *a);
	inline void diff3(const double *a, const double *b, double *out);
	inline void mult3(double *v, double s);
	inline double sqr_dist3(const double a[3], const double b[3]);

	inline void add_tensor_product_to_mat3x3(const double* a, const double* b, double mat[3][3]);
	inline void mult3x3(const double mat[3][3], const double v[3], double out[3]);
	inline void set3x3(double m[3][3], double s);
	inline void sub3x3(double res[3][3], const double m[3][3]);
	inline void tensor_product3x3(const double* a, const double* b, double out[3][3]);

protected:
	vtkPointLocator *mPointLocator;
	vtkPolyData *mTarget;
	double mEpsilon;
};

//=== inline methods =========================================================================================================================

inline void ObjRecICP::setTarget(vtkPolyData* target)
{
	mTarget = target;
	// Build the fast locator
	if ( mPointLocator ) mPointLocator->Delete();
	mPointLocator = vtkPointLocator::New();
	mPointLocator->SetDataSet(target);
	mPointLocator->BuildLocator();
}

//============================================================================================================================================

inline void ObjRecICP::add3(double *v, const double *a)
{
	v[0] += a[0];
	v[1] += a[1];
	v[2] += a[2];
}

//============================================================================================================================================

inline void ObjRecICP::diff3(const double *a, const double *b, double *out)
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

//============================================================================================================================================

inline void ObjRecICP::mult3(double *v, double s)
{
	v[0] *= s;
	v[1] *= s;
	v[2] *= s;
}

//============================================================================================================================================

#define ICP_SQR(x) ((x)*(x))

inline double ObjRecICP::sqr_dist3(const double a[3], const double b[3])
{
	return ICP_SQR(a[0]-b[0]) + ICP_SQR(a[1]-b[1]) + ICP_SQR(a[2]-b[2]);
}

//============================================================================================================================================

inline void ObjRecICP::add_tensor_product_to_mat3x3(const double* a, const double* b, double mat[3][3])
{
	mat[0][0] += a[0]*b[0]; mat[0][1] += a[0]*b[1]; mat[0][2] += a[0]*b[2];
	mat[1][0] += a[1]*b[0]; mat[1][1] += a[1]*b[1]; mat[1][2] += a[1]*b[2];
	mat[2][0] += a[2]*b[0]; mat[2][1] += a[2]*b[1]; mat[2][2] += a[2]*b[2];
}

//============================================================================================================================================

inline void ObjRecICP::mult3x3(const double mat[3][3], const double v[3], double out[3])
{
	out[0] = v[0]*mat[0][0] + v[1]*mat[0][1] + v[2]*mat[0][2];
	out[1] = v[0]*mat[1][0] + v[1]*mat[1][1] + v[2]*mat[1][2];
	out[2] = v[0]*mat[2][0] + v[1]*mat[2][1] + v[2]*mat[2][2];
}

//============================================================================================================================================

inline void ObjRecICP::set3x3(double m[3][3], double s)
{
	m[0][0] = m[0][1] = m[0][2] =
	m[1][0] = m[1][1] = m[1][2] =
	m[2][0] = m[2][1] = m[2][2] = s;
}

//============================================================================================================================================

inline void ObjRecICP::tensor_product3x3(const double* a, const double* b, double out[3][3])
{
	out[0][0] = a[0]*b[0]; out[0][1] = a[0]*b[1]; out[0][2] = a[0]*b[2];
	out[1][0] = a[1]*b[0]; out[1][1] = a[1]*b[1]; out[1][2] = a[1]*b[2];
	out[2][0] = a[2]*b[0]; out[2][1] = a[2]*b[1]; out[2][2] = a[2]*b[2];
}

//============================================================================================================================================

inline void ObjRecICP::sub3x3(double res[3][3], const double m[3][3])
{
	res[0][0] -= m[0][0]; res[0][1] -= m[0][1]; res[0][2] -= m[0][2];
	res[1][0] -= m[1][0]; res[1][1] -= m[1][1]; res[1][2] -= m[1][2];
	res[2][0] -= m[2][0]; res[2][1] -= m[2][1]; res[2][2] -= m[2][2];
}

//============================================================================================================================================

inline void ObjRecICP::cvPolarDecomp(const double M[3][3], double R[3][3])
{
	cv::Mat mcv(3, 3, CV_64FC1), qcv(3, 3, CV_64FC1);

	mcv.at<double>(0, 0) = M[0][0]; mcv.at<double>(0, 1) = M[0][1]; mcv.at<double>(0, 2) = M[0][2];
	mcv.at<double>(1, 0) = M[1][0]; mcv.at<double>(1, 1) = M[1][1]; mcv.at<double>(1, 2) = M[1][2];
	mcv.at<double>(2, 0) = M[2][0]; mcv.at<double>(2, 1) = M[2][1]; mcv.at<double>(2, 2) = M[2][2];

	cv::SVD svd(mcv);
	qcv = svd.u*svd.vt;

	if ( cv::determinant(qcv) < 0.0 )
	{
		svd.vt.at<double>(2,0) = -svd.vt.at<double>(2,0);
		svd.vt.at<double>(2,1) = -svd.vt.at<double>(2,1);
		svd.vt.at<double>(2,2) = -svd.vt.at<double>(2,2);
		qcv = svd.u*svd.vt;
	}

	// Write back
	R[0][0] = qcv.at<double>(0, 0); R[0][1] = qcv.at<double>(0, 1); R[0][2] = qcv.at<double>(0, 2);
	R[1][0] = qcv.at<double>(1, 0); R[1][1] = qcv.at<double>(1, 1); R[1][2] = qcv.at<double>(1, 2);
	R[2][0] = qcv.at<double>(2, 0); R[2][1] = qcv.at<double>(2, 1); R[2][2] = qcv.at<double>(2, 2);
}

//============================================================================================================================================

#endif /* _OBJ_REC_ICP_H_ */
