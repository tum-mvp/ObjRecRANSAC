/*
 * SphericalBoxSampler.h
 *
 *  Created on: Apr 11, 2010
 *      Author: papazov
 */

#ifndef SPHERICALBOXSAMPLER_H_
#define SPHERICALBOXSAMPLER_H_

#include "PointSampler.h"
#include "SphericalPatchSampler.h"
#include "RandomGenerator.h"
#include "../ComputationalGeometry/Algorithms/AuxCompGeom.h"
#include <cmath>

class SphericalBoxSampler: public PointSampler
{
public:
	SphericalBoxSampler();
	virtual ~SphericalBoxSampler();

	/** Samples phi, psi and theta such that the resulting rotation is uniformly
	  * distributed in 'range'. Use 'this->sampleUniformlyCartesian()' is you want
	  * to have the rotation as point with Cartesian coordinates. */
	inline void sampleUniformly(const Box& range, double* point);

	inline void sampleUniformlyCartesian(const Box& range, double* point);

protected:
	RandomGenerator mRandGen;
	SphericalPatchSampler mSphericalPatchSampler;
	double mOneThird;
};

//=== inline methods ===================================================================================

inline void SphericalBoxSampler::sampleUniformly(const Box& range, double* point)
{
	// Sample phi and psi and save them in point[0] and point[1], respectively.
	mSphericalPatchSampler.sampleUniformly(range, point);
	double theta1_pow3 = pow(range.mIntervals[2][0], 3.0);

	// Sample theta
	point[2] = pow(
			mRandGen.getRandomNumberInUnitInterval()*(pow(range.mIntervals[2][1], 3.0) - theta1_pow3) + theta1_pow3,
			mOneThird);
}

//======================================================================================================

inline void SphericalBoxSampler::sampleUniformlyCartesian(const Box& range, double* point)
{
	this->sampleUniformly(range, point);
	AuxCompGeom::ballCoordinates2Cartesian(point[0]/*phi*/, point[1]/*psi*/, point[2]/*theta*/, point);
}

//======================================================================================================

#endif /* SPHERICALBOXSAMPLER_H_ */
