/*
 * PointSampler.h
 *
 *  Created on: Mar 30, 2010
 *      Author: papazov
 */

#ifndef POINTSAMPLER_H_
#define POINTSAMPLER_H_

#include "../DataStructures/Box.h"

class PointSampler
{
public:
	PointSampler();
	virtual ~PointSampler();

	/** Samples a point uniformly from 'range'. The sample is saved in 'point'.
	  * 'point' should have the same dimension as 'range'. */
	virtual void sampleUniformly(const Box& range, double* point) = 0;
};

#endif /* POINTSAMPLER_H_ */
