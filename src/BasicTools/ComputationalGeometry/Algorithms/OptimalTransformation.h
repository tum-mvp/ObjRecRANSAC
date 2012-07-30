#ifndef _TUM_OPTIMALTRANSFORMATION_H_
#define _TUM_OPTIMALTRANSFORMATION_H_

#include "../../LinearAlgebra/Matrix.h"
#include <cmath>
#include <list>

using namespace std;

namespace tum
{

class OptimalTransformation
{
public:
	OptimalTransformation();
	virtual ~OptimalTransformation();

	/** Computes the rigid transform in that best matches the line (a1, b1) to (a2, b2).
	 * The computation is based on the corresponding points 'a1' <-> 'a2' and 'b1' <-> 'b2'
	 * and	the normals 'a1_n', 'b1_n', 'a2_n', and 'b2_n'. The result is saved in
	 * homogeneous coordinates in 'homMat'. Thread safe. */
	void getRigidTransform(const double *a1, const double *a1_n, const double *b1, const double* b1_n,
			const double *a2, const double *a2_n, const double *b2, const double* b2_n, double** homMat) const;

	/** Computes the rigid transform in that best matches the line (a1, b1) to (a2, b2).
	 * The computation is based on the corresponding points 'a1' <-> 'a2' and 'b1' <-> 'b2'
	 * and	the normals 'a1_n', 'b1_n', 'a2_n', and 'b2_n'. The result is saved in 'rig'
	 * which is an array of length 12. The first 9 elements are the rotational part (row major order)
	 * and the last 3 are the translation. Thread safe. */
	void getRigidTransform(const double *a1, const double *a1_n, const double *b1, const double* b1_n,
			const double *a2, const double *a2_n, const double *b2, const double* b2_n, double* rig) const;

	/** Computes the optimal rotation (in least squares sense) that maps 'points1' to 'points2' and saves
	  * it in 'rotmat'. 'points1' and 'points2' have to be 'numOfPoints' x 3 matrices. All points in
	  * 'points1' and 'points2' have to be centered! The method assumes that the k-th point in 'points1'
	  * corresponds to the k-th point in 'points2' for all k in {0, 1, ..., 'numOfPoints'-1}. Thread safe. */
	void getOptimalRotation(const double** points1, const double** points2, int numOfPoints, double** rotmat) const;

	inline void getRotationMatrixX(double u, double** rotmat) const;
	inline void getRotationMatrixZ(double u, double** rotmat) const;
	inline void getRotationMatrixXYZ(double u, double v, double w, double** rotmat) const;
};

}//namespace tum

//=== inline methods ============================================================================================================

inline void tum::OptimalTransformation::getRotationMatrixZ(double u, double** rotmat) const
{
	rotmat[0][0] = cos(u); rotmat[0][1] = -sin(u); rotmat[0][2] = 0.0;
	rotmat[1][0] = sin(u); rotmat[1][1] =  cos(u); rotmat[1][2] = 0.0;
	rotmat[2][0] = 0.0;    rotmat[2][1] =  0.0;    rotmat[2][2] = 1.0;
}

//===============================================================================================================================

inline void tum::OptimalTransformation::getRotationMatrixX(double u, double** rotmat) const
{
	rotmat[0][0] = 1.0; rotmat[0][1] = 0.0;	   rotmat[0][2] =  0.0;
	rotmat[1][0] = 0.0; rotmat[1][1] = cos(u); rotmat[1][2] = -sin(u);
	rotmat[2][0] = 0.0; rotmat[2][1] = sin(u); rotmat[2][2] =  cos(u);
}

//===============================================================================================================================

inline void tum::OptimalTransformation::getRotationMatrixXYZ(double u, double v, double w, double** rotmat) const
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

#endif /*_TUM_OPTIMALTRANSFORMATION_H_*/
