#ifndef RANSACPLANEDETECTOR_H_
#define RANSACPLANEDETECTOR_H_

#include <vtkPoints.h>
#include <vtkIdList.h>
#include "../../Stochastics/RandomGenerator.h"
#include <list>

using namespace std;

#define RANSAC_PLANE_DETECT_PRINT


class RANSACPlaneDetector
{
public:
	RANSACPlaneDetector();
	virtual ~RANSACPlaneDetector();

	void detectPlane(vtkPoints* input, double relNumOfPlanePoints, double planeThickness);

	/** Check the normal of the detected plane and make sure that it points to the right direction. Points which lie
	  * on the side of the plane towards the normal are considered as "above the plane". */
	void getPointsAbovePlane(list<int>& abovePlanePointIds, list<int>& belowPlanePointIds);

	/** Check the normal of the detected plane and make sure that it points to the right direction. Points which lie
	  * on the side of the plane towards the normal are considered as "above the plane". */
	void getPointsAbovePlane(vtkPoints* abovePoints, vtkPoints* belowPoints);

	/** Points which are close to the detected plane (closer then 0.5*plane_thickness) will be saved in 'onPoints' and the rest in 'offPoints'. */
	void getPointsOnPlane(vtkPoints* onPoints, vtkPoints* offPoints);

	void setPlaneThickness(double thickness){ mMaxPlaneDist = 0.5*thickness;}
	inline const double* getPlaneNormal(){ return mBestPlaneNormal;}
	inline void flipPlaneNormal();
	inline void getPlanePoints(double p1[3], double p2[3], double p3[3]);

	void setSuccessProbability(double value){ mSuccessProbability = value;}
	double getSuccessProbability(){ return mSuccessProbability;}

protected:
	inline int computeNumberOfIterations(int numberOfPoints, int numberOfPlanePoints, double successProbabilityToAchieve);
	inline int compPlaneSupport(vtkPoints* points, const double p1[3], const double p2[3], const double p3[3]);
	inline void randomlyGet3Points(vtkPoints* points, double p1[3], double p2[3], double p3[3]);

protected:
	RandomGenerator mRandGen;
	double mSuccessProbability, mMaxPlaneDist, mPlanePoint1[3], mPlanePoint2[3], mPlanePoint3[3], mBestPlaneNormal[3];
	vtkPoints* mInputPoints;
};

//=== inline methods ===========================================================================================================

inline void RANSACPlaneDetector::flipPlaneNormal()
{
	mBestPlaneNormal[0] = -mBestPlaneNormal[0];
	mBestPlaneNormal[1] = -mBestPlaneNormal[1];
	mBestPlaneNormal[2] = -mBestPlaneNormal[2];
}

//==============================================================================================================================

inline void RANSACPlaneDetector::getPlanePoints(double p1[3], double p2[3], double p3[3])
{
	Vector::copy3(mPlanePoint1, p1);
	Vector::copy3(mPlanePoint2, p2);
	Vector::copy3(mPlanePoint3, p3);
}

//==============================================================================================================================

inline int RANSACPlaneDetector::computeNumberOfIterations(int numberOfPoints, int numberOfPlanePoints, double successProbabilityToAchieve)
{
	const double M = numberOfPlanePoints, N = numberOfPoints;
	double q = 1.0 - ((M-2.0)*(M-1.0)*M)/((N-2.0)*(N-1.0)*N);

	return (int)(log(1-successProbabilityToAchieve)/log(q)) + 1;
}

//==============================================================================================================================

inline void RANSACPlaneDetector::randomlyGet3Points(vtkPoints* points, double p1[3], double p2[3], double p3[3])
{
	// Some variables
	int id1, id2, id3, range[2] = {0, points->GetNumberOfPoints()-1};

	// Get first id
	id1 = mRandGen.getRandomInteger(range);
	// Get second id
	do {
		id2 = mRandGen.getRandomInteger(range);
	} while ( id2 == id1 );
	// Get third id
	do {
		id3 = mRandGen.getRandomInteger(range);
	} while ( id3 == id1 || id3 == id2 );

	points->GetPoint(id1, p1);
	points->GetPoint(id2, p2);
	points->GetPoint(id3, p3);
}

//==========================================================================================================================

inline int RANSACPlaneDetector::compPlaneSupport(vtkPoints* points, const double p1[3], const double p2[3], const double p3[3])
{
	int i, support;
	double p[3], v1[3], v2[3], n[3];

	// Compute the plane normal
	Vector::diff(p2, p1, v1);
	Vector::diff(p3, p1, v2);
	Vector::cross3(v1, v2, n);
	Vector::normalize3(n);

	for ( i = 0, support = 0 ; i < points->GetNumberOfPoints() ; ++i )
	{
		points->GetPoint(i, p);
		// Get the connecting line between the plane point p1 and p
		p[0] -= p1[0];
		p[1] -= p1[1];
		p[2] -= p1[2];
		// Compute the length of the projection of p to the plane normal n
		if ( fabs(Vector::dot3(p, n)) <= mMaxPlaneDist )
			++support;
	}

	return support;
}

//==============================================================================================================================

#endif /*RANSACPLANEDETECTOR_H_*/
