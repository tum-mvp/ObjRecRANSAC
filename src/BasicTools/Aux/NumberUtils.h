/*
 * NumberUtils.h
 *
 *  Created on: Dec 16, 2009
 *      Author: papazov
 */

#ifndef NUMBERUTILS_H_
#define NUMBERUTILS_H_

class NumberUtils
{
public:
	NumberUtils();
	virtual ~NumberUtils();

	/** Returns 2^'power'. */
	static int powerOf2(int power){ return 0x01 << power;}

	inline static int roundDouble(double x);

	inline static double getmax(const double* array, int length);
	inline static double getmin(const double* array, int length);
};

//=== inline methods ==============================================================================================================

inline int NumberUtils::roundDouble(double x)
{
	if ( x >= 0 )
		return (int)(x + 0.5);
	return (int)(x - 0.5);
}

//=================================================================================================================================

inline double NumberUtils::getmax(const double* array, int length)
{
	double max = array[0];

	for ( int i = 1 ; i < length ; ++i )
		if ( array[i] > max )
			max = array[i];

	return max;
}

//=================================================================================================================================

inline double NumberUtils::getmin(const double* array, int length)
{
	double min = array[0];

	for ( int i = 1 ; i < length ; ++i )
		if ( array[i] < min )
			min = array[i];

	return min;

}

//=================================================================================================================================

#endif /* NUMBERUTILS_H_ */
