/*
 * RigidTransformSampler.cpp
 *
 *  Created on: Mar 30, 2010
 *      Author: papazov
 */

#include "RigidTransformSampler.h"
#include "../Aux/NumberUtils.h"
#include "../ComputationalGeometry/Algorithms/AuxCompGeom.h"
#include "../LinearAlgebra/Matrix.h"
#include <cmath>

RigidTransformSampler::RigidTransformSampler()
{
}

RigidTransformSampler::~RigidTransformSampler()
{
}

//==========================================================================================================================================

void RigidTransformSampler::getCartesianBBoxOfSphericalBox(const double** range, double bounds[6])
{
	double dist1, dist2, halfpi = 0.5*M_PI, pi1_5 = M_PI*1.5, pi2_5 = M_PI*2.5;

	double x_phi_min, x_phi_max;
	// Compute min phi for x
	if ( range[0][0] < M_PI && range[0][1] > M_PI )
		x_phi_min = M_PI;
	else
	{
		if ( fabs(range[0][0] - M_PI) < fabs(range[0][1] - M_PI) ) x_phi_min = range[0][0];
		else x_phi_min = range[0][1];
	}
	// Compute max phi for x
	if ( range[0][0] < 2.0*M_PI - range[0][1] ) x_phi_max = range[0][0];
	else x_phi_max = range[0][1];

	double y_phi_min, y_phi_max;
	// Compute the min phi for y
	if ( range[0][0] < pi1_5 && range[0][1] > pi1_5 )
		y_phi_min = pi1_5;
	else
	{
		// Compute the distance between 'range[0][0]' and 1.5pi
		if ( range[0][0] <= halfpi ) dist1 = range[0][0] + halfpi;
		else dist1 = fabs(range[0][0] - pi1_5);

		// Compute the distance between 'range[0][1]' and 1.5pi
		if ( range[0][1] <= halfpi ) dist2 = range[0][1] + halfpi;
		else dist2 = fabs(range[0][1] - pi1_5);

		if ( dist1 < dist2 ) y_phi_min = range[0][0];
		else y_phi_min = range[0][1];
	}
	// Compute the max phi for y
	if ( range[0][0] < halfpi && range[0][1] > halfpi )
		y_phi_max = halfpi;
	else
	{
		if ( range[0][0] >= pi1_5 ) dist1 = pi2_5 - range[0][0];
		else dist1 = fabs(range[0][0] - halfpi);

		if ( range[0][1] >= pi1_5 ) dist2 = pi2_5 - range[0][1];
		else dist2 = fabs(range[0][1] - halfpi);

		if ( dist1 < dist2 ) y_phi_max = range[0][0];
		else y_phi_max = range[0][1];
	}

	double xy_psi_min, xy_psi_max;
	// Compute the max and min psi
	if ( fabs(range[1][0] - halfpi) < fabs(range[1][1] - halfpi) )
	{
		xy_psi_min = range[1][1];
		xy_psi_max = range[1][0];
	}
	else
	{
		xy_psi_min = range[1][0];
		xy_psi_max = range[1][1];
	}
	// Small check
	if ( range[1][0] < halfpi && range[1][1] > halfpi )
		xy_psi_max = halfpi;


	double p1[3], p2[3], array[4];
	// Get min for x
	AuxCompGeom::ballCoordinates2Cartesian(x_phi_min, xy_psi_min, range[2][0], p1);
	array[0] = p1[0];
	AuxCompGeom::ballCoordinates2Cartesian(x_phi_min, xy_psi_min, range[2][1], p1);
	array[1] = p1[0];
	AuxCompGeom::ballCoordinates2Cartesian(x_phi_min, xy_psi_max, range[2][0], p1);
	array[2] = p1[0];
	AuxCompGeom::ballCoordinates2Cartesian(x_phi_min, xy_psi_max, range[2][1], p1);
	array[3] = p1[0];
	// Get the minimum
	bounds[0] = NumberUtils::getmin(array, 4);

	// Get max for x
	AuxCompGeom::ballCoordinates2Cartesian(x_phi_max, xy_psi_min, range[2][0], p1);
	array[0] = p1[0];
	AuxCompGeom::ballCoordinates2Cartesian(x_phi_max, xy_psi_min, range[2][1], p1);
	array[1] = p1[0];
	AuxCompGeom::ballCoordinates2Cartesian(x_phi_max, xy_psi_max, range[2][0], p1);
	array[2] = p1[0];
	AuxCompGeom::ballCoordinates2Cartesian(x_phi_max, xy_psi_max, range[2][1], p1);
	array[3] = p1[0];
	// Get the maximum
	bounds[1] = NumberUtils::getmax(array, 4);

	// Get min for y
	AuxCompGeom::ballCoordinates2Cartesian(y_phi_min, xy_psi_min, range[2][0], p1);
	array[0] = p1[1];
	AuxCompGeom::ballCoordinates2Cartesian(y_phi_min, xy_psi_min, range[2][1], p1);
	array[1] = p1[1];
	AuxCompGeom::ballCoordinates2Cartesian(y_phi_min, xy_psi_max, range[2][0], p1);
	array[2] = p1[1];
	AuxCompGeom::ballCoordinates2Cartesian(y_phi_min, xy_psi_max, range[2][1], p1);
	array[3] = p1[1];
	// Get the minimum
	bounds[2] = NumberUtils::getmin(array, 4);

	// Get max for y
	AuxCompGeom::ballCoordinates2Cartesian(y_phi_max, xy_psi_min, range[2][0], p1);
	array[0] = p1[1];
	AuxCompGeom::ballCoordinates2Cartesian(y_phi_max, xy_psi_min, range[2][1], p1);
	array[1] = p1[1];
	AuxCompGeom::ballCoordinates2Cartesian(y_phi_max, xy_psi_max, range[2][0], p1);
	array[2] = p1[1];
	AuxCompGeom::ballCoordinates2Cartesian(y_phi_max, xy_psi_max, range[2][1], p1);
	array[3] = p1[1];
	// Get the maximum
	bounds[3] = NumberUtils::getmax(array, 4);

	// Get min for z
	AuxCompGeom::ballCoordinates2Cartesian(0.0, range[1][1], range[2][0], p1);
	AuxCompGeom::ballCoordinates2Cartesian(0.0, range[1][1], range[2][1], p2);
	if ( p1[2] < p2[2] ) bounds[4] = p1[2];
	else bounds[4] = p2[2];
	// Get max for z
	AuxCompGeom::ballCoordinates2Cartesian(0.0, range[1][0], range[2][0], p1);
	AuxCompGeom::ballCoordinates2Cartesian(0.0, range[1][0], range[2][1], p2);
	if ( p1[2] > p2[2] ) bounds[5] = p1[2];
	else bounds[5] = p2[2];
}

//==========================================================================================================================================
