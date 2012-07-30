/*
 * RigidTransformSampler.h
 *
 *  Created on: Mar 30, 2010
 *      Author: papazov
 */

#ifndef RIGIDTRANSFORMSAMPLER_H_
#define RIGIDTRANSFORMSAMPLER_H_

#include "../DataStructures/Box.h"
#include "SphericalBoxSampler.h"
#include "RandomGenerator.h"
#include "PointSampler.h"

class RigidTransformSampler: public PointSampler
{
public:
	RigidTransformSampler();
	virtual ~RigidTransformSampler();

	/** Samples a rigid transform given by a rotation and a translation.
	  * The first three intervals of 'range' define the range of the rotation as follows. The rotation is defined
	  * by an axis n and an angle of rotation theta about that axis. n is given in spherical coordinates as
	  * n = (cos(phi)sin(psi), sin(phi)sin(psi), cos(psi)), where
	  *   phi in [range.mIntervals[0][0], range.mIntervals[0][1]],
	  *   psi in [range.mIntervals[1][0], range.mIntervals[1][1]] and
	  * theta in [range.mIntervals[2][0], range.mIntervals[2][1]].
	  * The translation will be sampled from the 3D interval defined by the last intervals of 'range'.
	  * The distribution is uniformly in the space of rigid transforms and NOT in the parameter space!
	  * The sample is saved in 'point', where phi = 'point[0]', psi = 'point[1]', theta = 'point[2]'
	  * and the translation is given by (point[3], point[4], point[5]). */
	inline void sampleUniformly(const Box& range, double* point);

	void getCartesianBBoxOfSphericalBox(const double** range, double bounds[6]);

protected:
	SphericalBoxSampler mSphericalBoxSampler;
	RandomGenerator mRandGen;
};

//=== inline methods ===================================================================================================

inline void RigidTransformSampler::sampleUniformly(const Box& range, double* point)
{
	// Sample the rotation
	mSphericalBoxSampler.sampleUniformly(range, point);
	// Sample the translation
	mRandGen.sampleUniformlyFrom3dBox(range.mIntervals[3], range.mIntervals[4], range.mIntervals[5],
			point[3], point[4], point[5]);
}

//======================================================================================================================

#endif /* RIGIDTRANSFORMSAMPLER_H_ */
