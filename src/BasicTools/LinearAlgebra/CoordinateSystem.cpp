#include "CoordinateSystem.h"
#include "Matrix.h"
#include <cmath>

using namespace tum;

CoordinateSystem::CoordinateSystem()
{
	this->setToCanonical();
}

CoordinateSystem::~CoordinateSystem()
{
}

//========================================================================================================================================

void CoordinateSystem::print(FILE* fp, char* label)
{
	if ( label != NULL )
		fprintf(fp, "=== %s ===\n", label);

	fprintf(fp, "frame:\n%lf %lf %lf\n%lf %lf %lf\n%lf %lf %lf\norigin\n%lf %lf %lf\n",
			mFrame[0][0], mFrame[0][1], mFrame[0][2],
			mFrame[1][0], mFrame[1][1], mFrame[1][2],
			mFrame[2][0], mFrame[2][1], mFrame[2][2],
			mOrigin[0], mOrigin[1], mOrigin[2]);
}

//========================================================================================================================================

void CoordinateSystem::translate(double x, double y, double z)
{
	mOrigin[0] += x;
	mOrigin[1] += y;
	mOrigin[2] += z;
}

//========================================================================================================================================

void CoordinateSystem::translate(double t[3])
{
	mOrigin[0] += t[0];
	mOrigin[1] += t[1];
	mOrigin[2] += t[2];
}

//========================================================================================================================================

void CoordinateSystem::copyFrom(const CoordinateSystem& src)
{
	mFrame[0][0] = src.mFrame[0][0]; mFrame[0][1] = src.mFrame[0][1]; mFrame[0][2] = src.mFrame[0][2];
	mFrame[1][0] = src.mFrame[1][0]; mFrame[1][1] = src.mFrame[1][1]; mFrame[1][2] = src.mFrame[1][2];
	mFrame[2][0] = src.mFrame[2][0]; mFrame[2][1] = src.mFrame[2][1]; mFrame[2][2] = src.mFrame[2][2];

	mOrigin[0] = src.mOrigin[0];
	mOrigin[1] = src.mOrigin[1];
	mOrigin[2] = src.mOrigin[2];
}

//========================================================================================================================================

void CoordinateSystem::setToCanonical()
{
	mFrame[0][0] = 1.0; mFrame[0][1] = 0.0; mFrame[0][2] = 0.0;
	mFrame[1][0] = 0.0; mFrame[1][1] = 1.0; mFrame[1][2] = 0.0;
	mFrame[2][0] = 0.0; mFrame[2][1] = 0.0; mFrame[2][2] = 1.0;

	mOrigin[0] = mOrigin[1] = mOrigin[2] = 0.0;	
}

//========================================================================================================================================

void CoordinateSystem::setFrame(double mat[3][3])
{
	mFrame[0][0] = mat[0][0]; mFrame[0][1] = mat[0][1]; mFrame[0][2] = mat[0][2];
	mFrame[1][0] = mat[1][0]; mFrame[1][1] = mat[1][1]; mFrame[1][2] = mat[1][2];
	mFrame[2][0] = mat[2][0]; mFrame[2][1] = mat[2][1]; mFrame[2][2] = mat[2][2];
}

//========================================================================================================================================

void CoordinateSystem::setFrame(const double **mat)
{
	mFrame[0][0] = mat[0][0]; mFrame[0][1] = mat[0][1]; mFrame[0][2] = mat[0][2];
	mFrame[1][0] = mat[1][0]; mFrame[1][1] = mat[1][1]; mFrame[1][2] = mat[1][2];
	mFrame[2][0] = mat[2][0]; mFrame[2][1] = mat[2][1]; mFrame[2][2] = mat[2][2];
}

//========================================================================================================================================

void CoordinateSystem::flipZAxis()
{
	mFrame[0][1] *= -1; mFrame[0][2] *= -1;
	mFrame[1][1] *= -1; mFrame[1][2] *= -1;
	mFrame[2][1] *= -1; mFrame[2][2] *= -1;
}

//========================================================================================================================================

void CoordinateSystem::operator+=(const CoordinateSystem& cs)
{
	mFrame[0][0] += cs.mFrame[0][0]; mFrame[0][1] += cs.mFrame[0][1]; mFrame[0][2] += cs.mFrame[0][2];
	mFrame[1][0] += cs.mFrame[1][0]; mFrame[1][1] += cs.mFrame[1][1]; mFrame[1][2] += cs.mFrame[1][2];
	mFrame[2][0] += cs.mFrame[2][0]; mFrame[2][1] += cs.mFrame[2][1]; mFrame[2][2] += cs.mFrame[2][2];

	mOrigin[0] += cs.mOrigin[0];
	mOrigin[1] += cs.mOrigin[1];
	mOrigin[2] += cs.mOrigin[2];
}

//========================================================================================================================================

void CoordinateSystem::normalize()
{
	double len;
	len = sqrt(mFrame[0][0]*mFrame[0][0] + mFrame[1][0]*mFrame[1][0] + mFrame[2][0]*mFrame[2][0]);
	mFrame[0][0] /= len;
	mFrame[1][0] /= len;
	mFrame[2][0] /= len;
	len = sqrt(mFrame[0][1]*mFrame[0][1] + mFrame[1][1]*mFrame[1][1] + mFrame[2][1]*mFrame[2][1]);
	mFrame[0][1] /= len;
	mFrame[1][1] /= len;
	mFrame[2][1] /= len;
	len = sqrt(mFrame[0][2]*mFrame[0][2] + mFrame[1][2]*mFrame[1][2] + mFrame[2][2]*mFrame[2][2]);
	mFrame[0][2] /= len;
	mFrame[1][2] /= len;
	mFrame[2][2] /= len;
}

//========================================================================================================================================

void CoordinateSystem::multFrameFromRight(double mat[3][3])
{
	double tmp[3][3];
	Matrix::mult3x3(mFrame, mat, tmp);
	Matrix::copy3x3(tmp, mFrame);
}

//========================================================================================================================================
