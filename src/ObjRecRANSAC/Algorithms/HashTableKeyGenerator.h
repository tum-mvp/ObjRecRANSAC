#ifndef HASHTABLEKEYGENERATOR_H_
#define HASHTABLEKEYGENERATOR_H_

#include <BasicTools/LinearAlgebra/Vector.h>
#include <cmath>

using namespace tum;

class HashTableKeyGenerator
{
public:
	HashTableKeyGenerator(){}
	virtual ~HashTableKeyGenerator(){}

	inline void computeHashTableKey3(const double* p1, const double* n1, const double* p2, const double* n2, double* key)const;
};

//=== inline methods =============================================================================================================

inline void HashTableKeyGenerator::computeHashTableKey3(const double* p1, const double* n1,
		const double* p2, const double* n2, double* key)const
{
	double line[3];
	// Get the line from p1 to p2
	Vector::diff(p2, p1, line);
	Vector::normalize3(line);

	// Compute the angle between the connecting line and the first normal
	key[0] = acos(Vector::dot3(n1, line));
	// Compute the angle between the connecting line and the second normal
	line[0] *= -1.0; line[1] *= -1.0; line[2] *= -1.0;
	key[1] = acos(Vector::dot3(n2, line));
	// Compute the angle between both normals
	key[2] = acos(Vector::dot3(n1, n2));
}

//================================================================================================================================

#endif /*HASHTABLEKEYGENERATOR_H_*/
