#include "RANSACPlaneDetector.h"
#include <cmath>
#include <cstdio>


RANSACPlaneDetector::RANSACPlaneDetector()
{
	mSuccessProbability = 0.9;
}

RANSACPlaneDetector::~RANSACPlaneDetector()
{
}

//==========================================================================================================================

void RANSACPlaneDetector::detectPlane(vtkPoints* input, double relNumOfPlanePoints, double planeThickness)
{
	mInputPoints = input;
	mMaxPlaneDist = 0.5*planeThickness;

	// Compute the number of iterations
	int numOfIterations = this->computeNumberOfIterations(
			input->GetNumberOfPoints(),
			(int)(relNumOfPlanePoints*(double)input->GetNumberOfPoints() + 0.5),
			mSuccessProbability);

	// Some variables
	double p1[3], p2[3], p3[3];

	// Initialization run
	this->randomlyGet3Points(input, mPlanePoint1, mPlanePoint2, mPlanePoint3);
	int i, support, max_support = this->compPlaneSupport(input, mPlanePoint1, mPlanePoint2, mPlanePoint3);

#ifdef RANSAC_PLANE_DETECT_PRINT
	printf("RANSACPlaneDetector::%s(): detecting plane ...\n", __func__);
	double factor = 100.0/(double)numOfIterations;
#endif

	for ( i = 1 ; i < numOfIterations ; ++i )
	{
		this->randomlyGet3Points(input, p1, p2, p3);
		support = this->compPlaneSupport(input, p1, p2, p3);
		// Check if we have a better support
		if ( support > max_support )
		{
			Vector::copy3(p1, mPlanePoint1);
			Vector::copy3(p2, mPlanePoint2);
			Vector::copy3(p3, mPlanePoint3);
			max_support = support;
		}
#ifdef RANSAC_PLANE_DETECT_PRINT
		printf("\r%.1lf%% [plane support = %i] ", (double)i*factor, max_support); fflush(stdout);
#endif
	}

#ifdef RANSAC_PLANE_DETECT_PRINT
	printf("\r%.1lf%% done [plane support = %i]\n", (double)i*factor, max_support); fflush(stdout);
#endif

	// Compute the normal of the detected plane
	Vector::diff(mPlanePoint2, mPlanePoint1, p2);
	Vector::diff(mPlanePoint3, mPlanePoint1, p3);
	Vector::cross3(p2, p3, mBestPlaneNormal);
	Vector::normalize3(mBestPlaneNormal);
}

//==========================================================================================================================

void RANSACPlaneDetector::getPointsAbovePlane(list<int>& abovePlanePointIds, list<int>& belowPlanePointIds)
{
	double slen, p[3];
	// Determine which lie above the plane
	for ( int i = 0 ; i < mInputPoints->GetNumberOfPoints() ; ++i )
	{
		mInputPoints->GetPoint(i, p);
		// Get the connecting line between the plane point 'mPlanePoint1' and 'p'
		p[0] -= mPlanePoint1[0];
		p[1] -= mPlanePoint1[1];
		p[2] -= mPlanePoint1[2];
		// Compute the signed "length" of the projection of 'p' to the plane normal 'mBestPlaneNormal'
		slen = Vector::dot3(p, mBestPlaneNormal);

		// Check if the point is too close to the plane or it is on the "negative" side of the plane
		if ( slen <= mMaxPlaneDist )
		{
			belowPlanePointIds.push_back(i);
			continue;
		}

		// The point is above the plane
		abovePlanePointIds.push_back(i);
	}
}

//==========================================================================================================================

void RANSACPlaneDetector::getPointsAbovePlane(vtkPoints* abovePoints, vtkPoints* belowPoints)
{
	double slen, p[3], line[3];
	// Determine which point lies above the plane
	for ( int i = 0 ; i < mInputPoints->GetNumberOfPoints() ; ++i )
	{
		mInputPoints->GetPoint(i, p);
		// Get the connecting line between the plane point 'mPlanePoint1' and 'p'
		line[0] = p[0] - mPlanePoint1[0];
		line[1] = p[1] - mPlanePoint1[1];
		line[2] = p[2] - mPlanePoint1[2];
		// Compute the signed "length" of the projection of 'p' to the plane normal 'mBestPlaneNormal'
		slen = Vector::dot3(line, mBestPlaneNormal);

		// Check if the point is too close to the plane or it is on the "negative" side of the plane
		if ( slen <= mMaxPlaneDist )
		{
			belowPoints->InsertNextPoint(p);
			continue;
		}

		// The point is above the plane
		abovePoints->InsertNextPoint(p);
	}
}

//==========================================================================================================================

void RANSACPlaneDetector::getPointsOnPlane(vtkPoints* onPoints, vtkPoints* offPoints)
{
	double p[3], line[3];

	// Determine which point lies on the plane
	for ( int i = 0 ; i < mInputPoints->GetNumberOfPoints() ; ++i )
	{
		mInputPoints->GetPoint(i, p);
		// Get the connecting line between the plane point 'mPlanePoint1' and 'p'
		line[0] = p[0] - mPlanePoint1[0];
		line[1] = p[1] - mPlanePoint1[1];
		line[2] = p[2] - mPlanePoint1[2];

		// Check if the point is close enough to the plane
		if ( fabs(Vector::dot3(line, mBestPlaneNormal)) <= mMaxPlaneDist )
			onPoints->InsertNextPoint(p); // Yes, it's close enough
		else
			offPoints->InsertNextPoint(p); // Too far away
	}
}

//==========================================================================================================================

