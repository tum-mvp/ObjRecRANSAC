/*
 * AuxCompGeom.h
 *
 *  Created on: Jan 12, 2010
 *      Author: papazov
 */

#ifndef AUXCOMPGEOM_H_
#define AUXCOMPGEOM_H_

#include <cmath>
#include "../../LinearAlgebra/Vector.h"

using namespace tum;

class AuxCompGeom
{
public:
	AuxCompGeom();
	virtual ~AuxCompGeom();

	/** The bounding box of the 'points' is computed and saved in 'bbox', which has to be a 'dimension' x 2 matrix.
	  * 'points' has to be a 'numOfPoints' x 'dimension' matrix. */
	static void boundingBox(int numOfPoints, double** points, int dimension, double** bbox);

	/** Computes the bounds of 'points' and saves them in 'bounds'. */
	inline static void getBounds3d(int numOfPoints, const double** points, double bounds[6]);

	inline static double getDiagonalLength(const double min[3], const double max[3]);

	/** Returns 'true' if 'point' lies within the spherical box defined by 'range'. */
	inline static bool insideSphericalBox(const double** range, const double* point);

	/** Enlarges the 'box' by subtracting respectively adding to the lower respectively upper bound of each 'box' side
	  * the side length multiplied by 'factor'. 'box' has to be a 'dimension' by 2 matrix. */
	static void enlarge(double** box, int dimension, double factor);

	inline static void ballCoordinates2Cartesian(double phi, double psi, double r, double out[3]);

public:
	static const double mTwoPi;
};

//=== inline methods ==========================================================================================================================

inline void AuxCompGeom::ballCoordinates2Cartesian(double phi, double psi, double r, double out[3])
{
	out[0] = r*sin(psi)*cos(phi);
	out[1] = r*sin(psi)*sin(phi);
	out[2] = r*cos(psi);
}

//=============================================================================================================================================

inline bool AuxCompGeom::insideSphericalBox(const double** range, const double* point)
{
	double len = Vector::length3(point);
	// Check the length
	if ( len < range[2][0] || len >= range[2][1] ) return false;

	double psi = acos(point[2]/len);
	// Check psi
	if ( psi < range[1][0] || psi >= range[1][1] ) return false;

	double phi = atan2(point[1], point[0]);
	if ( phi < 0.0 )
		phi += AuxCompGeom::mTwoPi;

	// Check phi
	if ( phi < range[0][0] || phi >= range[0][1] ) return false;

	// The point passed all tests
	return true;
}

//=============================================================================================================================================

inline void AuxCompGeom::getBounds3d(int numOfPoints, const double** points, double bounds[6])
{
	// Initialization run with the first point
	bounds[0] = bounds[1] = points[0][0];
	bounds[2] = bounds[3] = points[0][1];
	bounds[4] = bounds[5] = points[0][2];

	// The real run for the rest of the points
	for ( int i = 1 ; i < numOfPoints ; ++i )
	{
		// The x-axis
		     if ( points[i][0] < bounds[0] ) bounds[0] = points[i][0];
		else if ( points[i][0] > bounds[1] ) bounds[1] = points[i][0];
		// The y-axis
			 if ( points[i][1] < bounds[2] ) bounds[2] = points[i][1];
		else if ( points[i][1] > bounds[3] ) bounds[3] = points[i][1];
		// The z-axis
			 if ( points[i][2] < bounds[4] ) bounds[4] = points[i][2];
		else if ( points[i][2] > bounds[5] ) bounds[5] = points[i][2];
	}
}

//=============================================================================================================================================

inline double AuxCompGeom::getDiagonalLength(const double min[3], const double max[3])
{
	return sqrt(pow(max[0]-min[0],2) + pow(max[1]-min[1],2) + pow(max[2]-min[2],2));
}

//=============================================================================================================================================

#endif /* AUXCOMPGEOM_H_ */
