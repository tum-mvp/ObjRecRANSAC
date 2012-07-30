#ifndef _TUM_COORDINATESYSTEM_H_
#define _TUM_COORDINATESYSTEM_H_

#include <cstdio>

namespace tum
{

class CoordinateSystem
{
public:
	CoordinateSystem();
	virtual ~CoordinateSystem();

	void setToCanonical();
	void setFrame(double mat[3][3]);
	void setFrame(const double **mat);
	void translate(double t[3]);
	void translate(double x, double y, double z);
	void setOrigin(double o[3]){ mOrigin[0] = o[0]; mOrigin[1] = o[1]; mOrigin[2] = o[2];}
	void setOrigin(double x, double y, double z){ mOrigin[0] = x; mOrigin[1] = y; mOrigin[2] = z;}
	void normalize();

	void multFrameFromRight(double mat[3][3]);

	void flipZAxis();
	void copyFrom(const CoordinateSystem& src);

	void operator+=(const CoordinateSystem& cs);
	void print(FILE* fp, char* label = NULL);

public:
	double mFrame[3][3], mOrigin[3];
};

}//namespace tum

#endif /*_TUM_COORDINATESYSTEM_H_*/
