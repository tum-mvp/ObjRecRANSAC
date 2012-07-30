/*
 * AuxCompGeom.cpp
 *
 *  Created on: Jan 12, 2010
 *      Author: papazov
 */

#include "AuxCompGeom.h"

#include <cmath>

const double AuxCompGeom::mTwoPi = 2.0*M_PI;

AuxCompGeom::AuxCompGeom()
{
}

AuxCompGeom::~AuxCompGeom()
{
}

//==============================================================================================================================

void AuxCompGeom::boundingBox(int numOfPoints, double** points, int dimension, double** bbox)
{
	int i, d;

	// Initialization run with the first point
	for ( d = 0 ; d < dimension ; ++d )
		bbox[d][0] = bbox[d][1] = points[0][d];

	// The real run for the rest of the points
	for ( d = 0 ; d < dimension ; ++d )
	{
		for ( i = 1 ; i < numOfPoints ; ++i )
		{
			if ( points[i][d] < bbox[d][0] )
				bbox[d][0] = points[i][d];
			else if ( points[i][d] > bbox[d][1] )
				bbox[d][1] = points[i][d];
		}
	}
}

//==============================================================================================================================

void AuxCompGeom::enlarge(double** box, int dimension, double factor)
{
	double s;

	for ( int i = 0 ; i < dimension ; ++i )
	{
		s = (box[i][1] - box[i][0])*factor;
		box[i][0] -= s;
		box[i][1] += s;
	}
}

//==============================================================================================================================
