#ifndef _TUM_NOISE_H_
#define _TUM_NOISE_H_

#include "RandomGenerator.h"
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <cmath>

namespace tum
{

class Noise
{
public:
	Noise();
	virtual ~Noise();

	/** Corrupts all points, by adding to their x, y, and z component a random number drawn uniformly within the intervals
	  * xbounds, ybounds, and zbounds respectively. */
	void corruptXYZ(vtkPoints* points, double xbounds[2], double ybounds[2], double zbounds[2]);
	/** Corrupts all points, by adding to their z component a random number uniformly drawn in [min, max]. */
	void corruptZ(vtkPoints* points, double min, double max);

	/** Adds zero mean Gaussian noise with variance 'sigmaAsPercentFromBBDiag' to the 'points'.
	 * 'sigmaAsPercentFromBBDiag' is the sigma of the Gaussian noise in percentage from the diagonal of the
	 * bounding box of the input points. */
	void addGaussianNoise(vtkPoints* points, double sigmaAsPercentFromBBDiag);

	/** Adds randomly numOfOutliers points uniformly distributed over the bounding box of points. */
	void addOutliers(vtkPoints* points, int numOfOutliers);
	/** Adds percentOutliers percent of points uniformly distributed over the bounding box of points.
	  * The percentage is from the original number of points. */
	inline void addOutliers(vtkPoints* points, double percentOutliers);

	/** Returns a zero-mean Gaussian sample with variance 1. */
	inline double getGaussianSample();

	inline void getBounds(vtkPoints* points, double bounds[6]);

protected:
	static bool mRandomGeneratorInit;
	RandomGenerator mRandGenerator;
};

}//namespace tum

//=== inline methods ==============================================================================================

double tum::Noise::getGaussianSample()
{
	double x1, x2;
	// Generate a random number in (0, 1]
	do {
		x1 = mRandGenerator.getRandomNumberInUnitInterval();
	} while ( x1 <= 0.0 );
	// Generate a random number in [0, 1]
	x2 = mRandGenerator.getRandomNumberInUnitInterval();

	// Return the Gaussian sample
	return sqrt(-log(x1))*sqrt(2.0)*cos(2.0*M_PI*x2);
}

//=================================================================================================================

inline void tum::Noise::getBounds(vtkPoints* points, double bounds[6])
{
	double p[3];
	// Init
	points->GetPoint(0, p);
	bounds[0] = bounds[1] = p[0];
	bounds[2] = bounds[3] = p[1];
	bounds[4] = bounds[5] = p[2];

	int i, numOfPoints = points->GetNumberOfPoints();
	for ( i = 1 ; i < numOfPoints ; ++i )
	{
		points->GetPoint(i, p);

		if ( p[0] < bounds[0] )
			bounds[0] = p[0];
		else if ( p[0] > bounds[1] )
			bounds[1] = p[0];

		if ( p[1] < bounds[2] )
			bounds[2] = p[1];
		else if ( p[1] > bounds[3] )
			bounds[3] = p[1];

		if ( p[2] < bounds[4] )
			bounds[4] = p[2];
		else if ( p[2] > bounds[5] )
			bounds[5] = p[2];
	}
}

//=================================================================================================================

inline void tum::Noise::addOutliers(vtkPoints* points, double percentOutliers)
{
	this->addOutliers( points, (int)((percentOutliers/100.0)*(double)points->GetNumberOfPoints()) );
}

//=================================================================================================================

#endif /*_TUM_NOISE_H_*/
