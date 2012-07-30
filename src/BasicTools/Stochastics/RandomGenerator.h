#ifndef _TUM_RANDOMGENERATOR_H_
#define _TUM_RANDOMGENERATOR_H_

#include "PointSampler.h"
#include "../DataStructures/Box.h"
#include "../LinearAlgebra/Vector.h"
#include "../LinearAlgebra/Matrix.h"
#include <cstdlib>
#include <cmath>
#include <vector>

using namespace std;

namespace tum
{

class RandomGenerator: public PointSampler
{
public:
	RandomGenerator();
	virtual ~RandomGenerator();

	/** Inherited from 'PointSampler'. */
	inline void sampleUniformly(const Box& range, double* point);

	/** Draws uniformly a random point within the box defined by min and max. The dimension has to be defined in dim. */
	inline void getRandomPointWithinBox(double *min, double *max, double *point, int dim);
	inline void getRandom3dPointWithinBox(const double bounds[6], double p[3]);
	inline void sampleUniformlyFrom3dBox(const double* intx, const double* inty, const double* intz,
			double& x, double& y, double& z);
	inline void getRandomPointWithinBox(Box& box, Vector& p);
	inline double getRandomNumberInUnitInterval();
	inline void getRandomRotationAngles(double *u, double *v, double *w);
	inline double getRandomNumberInInterval(double min, double max);

	/** Samples a point at random from the triangle defined by 'p1', 'p2' and 'p3' and saves it in 'out'. */
	inline void getRandomPointInTriangle(const double *p1, const double *p2, const double *p3, double* out);

	/** Returns k with probability (intervals[k+1]-intervals[k]). 'intervals' has to be a strictly monotonically increasing
	 * sequence of numbers having the last number equals to 1.  */
	inline int drawRandomNumberFromIntervals(vector<double>& intervals);

	/** Returns a random integer in the closed interval range (i.e., including the boundaries). */
	inline int getRandomInteger(int range[2]);
	inline int getRandomInteger(int min, int max) const;

	inline void getRandomAdjacencyMatrix(Matrix& mat);

private:
	static bool randomGeneratorInitialized;
};

}//namespace tum

//=== inline methods =============================================================================================================

inline void tum::RandomGenerator::sampleUniformly(const Box& range, double* point)
{
	for ( int i = 0 ; i < range.mDimension ; ++i )
		point[i] = this->getRandomNumberInInterval(range.mIntervals[i][0], range.mIntervals[i][1]);
}

//================================================================================================================================

inline void tum::RandomGenerator::getRandomPointWithinBox(double *min, double *max, double *point, int dim)
{
	double randNum;
	for ( int i = 0 ; i < dim ; ++i )
	{
		// A random number in [0, 1]
		randNum = (double)rand()/(double)RAND_MAX;
		// Generate the i-th random component of 'p'
		point[i] = min[i] + (max[i]-min[i])*randNum;
	}
}

//================================================================================================================================

inline void tum::RandomGenerator::getRandomPointWithinBox(Box& box, Vector& p)
{
	double randNum;
	for ( int i = 0 ; i < p.mDimension ; ++i )
	{
		// A random number in [0, 1]
		randNum = (double)rand()/(double)RAND_MAX;
		// Generate the i-th random component of 'p'
		p.mData[i] = box.mIntervals[i][0] + randNum*box.getDiameter(i);
	}
}

//================================================================================================================================

inline void tum::RandomGenerator::getRandom3dPointWithinBox(const double bounds[6], double p[3])
{
	p[0] = this->getRandomNumberInInterval(bounds[0], bounds[1]);
	p[1] = this->getRandomNumberInInterval(bounds[2], bounds[3]);
	p[2] = this->getRandomNumberInInterval(bounds[4], bounds[5]);
}

//================================================================================================================================

inline void tum::RandomGenerator::sampleUniformlyFrom3dBox(const double* intx, const double* inty, const double* intz,
		double& x, double& y, double& z)
{
	x = this->getRandomNumberInInterval(intx[0], intx[1]);
	y = this->getRandomNumberInInterval(inty[0], inty[1]);
	z = this->getRandomNumberInInterval(intz[0], intz[1]);
}

//================================================================================================================================

inline double tum::RandomGenerator::getRandomNumberInInterval(double min, double max)
{
	return min + (max-min)*((double)rand()/(double)RAND_MAX);
}

//================================================================================================================================

inline double tum::RandomGenerator::getRandomNumberInUnitInterval()
{
	return (double)rand()/(double)RAND_MAX;
}

//================================================================================================================================

inline int tum::RandomGenerator::getRandomInteger(int range[2])
{
	return (int)((double)range[0] + ((double)rand()/(double)RAND_MAX)*(double)(range[1]-range[0]) + 0.5);
}

//================================================================================================================================

inline int tum::RandomGenerator::getRandomInteger(int min, int max) const
{
	return (int)((double)min + ((double)rand()/(double)RAND_MAX)*(double)(max-min) + 0.5);
}

//================================================================================================================================

inline void tum::RandomGenerator::getRandomRotationAngles(double *u, double *v, double *w)
{
	// A number in [0, pi]
	*u = M_PI*((double)rand()/double(RAND_MAX));
	// A number in [0, 2pi]
	*v = 2.0*M_PI*((double)rand()/double(RAND_MAX));
	// A number in [0, 2pi]
	*w = 2.0*M_PI*((double)rand()/double(RAND_MAX));
}

//================================================================================================================================

inline void tum::RandomGenerator::getRandomPointInTriangle(const double *p1, const double *p2, const double *p3, double* out)
{
	double u, v;
	do
	{
		u = this->getRandomNumberInUnitInterval();
		v = this->getRandomNumberInUnitInterval();
	}
	while ( u + v > 1.0 );

	double v1[3], v2[3];
	// Compute the connecting vectors from 'p1' to the other two points
	Vector::diff(p2, p1, v1);
	Vector::diff(p3, p1, v2);
	// Multiply the connecting vectors by the two random numbers
	Vector::mult3(v1, u);
	Vector::mult3(v2, v);
	// The sum of 'v1' and 'v2'
	double vec[3];
	Vector::sum3(v1, v2, vec);
	// Compute the final point within the triangle
	Vector::sum3(p1, vec, out);
}

//================================================================================================================================

inline int tum::RandomGenerator::drawRandomNumberFromIntervals(vector<double>& intervals)
{
	double r = this->getRandomNumberInUnitInterval();
	int i, size = (unsigned int)intervals.size();

	for ( i = 0 ; i < size ; ++i )
		if ( r <= intervals[i] )
			break;

	return i;
}

//===================================================================================================================================

inline void tum::RandomGenerator::getRandomAdjacencyMatrix(Matrix& mat)
{
	int i, j;
	for ( i = 0 ; i < mat.getNumberOfRows() ; ++i )
		for ( j = i ; j < mat.getNumberOfColumns() ; ++j )
			mat.m[j][i] = mat.m[i][j] = (double)this->getRandomInteger(0, 1);
}

//================================================================================================================================

#endif /*_TUM_RANDOMGENERATOR_H_*/
