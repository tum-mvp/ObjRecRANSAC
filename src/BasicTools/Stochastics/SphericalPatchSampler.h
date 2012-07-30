/*
 * SphericalPatchSampler.h
 *
 *  Created on: Apr 11, 2010
 *      Author: papazov
 */

#ifndef SPHERICALPATCHSAMPLER_H_
#define SPHERICALPATCHSAMPLER_H_

#include "PointSampler.h"
#include "RandomGenerator.h"
#include "../ComputationalGeometry/Algorithms/AuxCompGeom.h"
#include <cmath>

class SphericalPatchSampler: public PointSampler
{
public:
	SphericalPatchSampler();
	virtual ~SphericalPatchSampler();

	/** Inherited from 'PointSampler'. Samples a point uniformly from the spherical patch defined
	  * by 'range'. The spherical patch is defined in standard spherical coordinates: phi is the angle
	  * with the positive x-axis and psi the angle with the positive z-axis. */
	inline void sampleUniformly(const Box& range, double* point);

	inline void sampleUniformlyCartesian(const Box& range, double* point);

	/** Only relevant if using 'this->sampleUniformlyCartesian()'. */
	void setSphereRadius(double r){ mRadius = r;}

protected:
	double mRadius;
	RandomGenerator mRandGen;
};

//=== inline methods ================================================================================================

inline void SphericalPatchSampler::sampleUniformly(const Box& range, double* point)
{
	// Sample phi
	point[0] = mRandGen.getRandomNumberInInterval(range.mIntervals[0][0], range.mIntervals[0][1]);
	double cos_psi1 = cos(range.mIntervals[1][0]);
	// Sample psi
	point[1] = acos(cos_psi1 - mRandGen.getRandomNumberInUnitInterval()*(cos_psi1 - cos(range.mIntervals[1][1])));
}

//===================================================================================================================

inline void SphericalPatchSampler::sampleUniformlyCartesian(const Box& range, double* point)
{
	this->sampleUniformly(range, point);
	AuxCompGeom::ballCoordinates2Cartesian(point[0]/*phi*/, point[1]/*psi*/, mRadius, point);
}

//===================================================================================================================

#endif /* SPHERICALPATCHSAMPLER_H_ */
