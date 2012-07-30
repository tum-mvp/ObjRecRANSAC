#include "AnalyticGeometry.h"
#include "../Stochastics/RandomGenerator.h"
#include "Vector.h"
#include <cmath>

using namespace tum;

AnalyticGeometry::AnalyticGeometry()
{
}

AnalyticGeometry::~AnalyticGeometry()
{
}

//===============================================================================================================================

bool AnalyticGeometry::getIntersectionPoint(const double* a, const double* b, const double* c, const double* d,
			double& r1, double& r2, double& dist, double* x)
{
	// Some auxiliary vectors
	double g[3], v[3], w[3];
	// Init
	Vector::diff(a, c, g); // g = a - c
	Vector::diff(b, a, v); // v = b - a
	Vector::diff(d, c, w); // w = d - c

	// Some auxiliary variables
	double v2 = Vector::dot3(v, v), w2 = Vector::dot3(w, w), vw = Vector::dot3(v, w);
	double v2w2 = v2*w2, vw2 = pow(vw, 2);

	// Check if v and w are parallel
	double B = v2w2/(v2w2 - vw2);

	if ( isinf(B) || isnan(B) )
		return false;

	double gv = Vector::dot3(g,v);
	// v and w are not parallel
	r2 = B*((v2*Vector::dot3(g,w) - gv*vw) / v2w2);
	r1 = (r2*vw - gv)/v2;

	// Compute x
	x[0] = (a[0]+r1*v[0] + c[0]+r2*w[0])/2.0;
	x[1] = (a[1]+r1*v[1] + c[1]+r2*w[1])/2.0;
	x[2] = (a[2]+r1*v[2] + c[2]+r2*w[2])/2.0;

	// Compute the distance between x and the lines
	double dvec[3];
	dvec[0] = g[0] + r1*v[0] - r2*w[0];
	dvec[1] = g[1] + r1*v[1] - r2*w[1];
	dvec[2] = g[2] + r1*v[2] - r2*w[2];
	dist = Vector::length3(dvec);

	return true;
}

//===============================================================================================================================

double AnalyticGeometry::signedDistanceToPlane3(const double a[3], const double b[3], const double c[3], const double p[3])
{
	double n1[3] = {b[0]-a[0], b[1]-a[1], b[2]-a[2]},
           n2[3] = {c[0]-a[0], c[1]-a[1], c[2]-a[2]},
            x[3] = {p[0]-a[0], p[1]-a[1], p[2]-a[2]}, n[3];
	Vector::cross3(n1, n2, n);
	Vector::normalize3(n);

	return Vector::dot3(n, x);
}

//===============================================================================================================================

void AnalyticGeometry::computeOrthonormalFrameForPlane3(const double* p, const double* n, double** frame)
{
	RandomGenerator rgen;
	double q[3], x[3], y[3], proj[3];

	do
	{
		// Get a random point "around" the plane point 'p'
		q[0] = rgen.getRandomNumberInInterval(-1.0, 1.0) + p[0];
		q[1] = rgen.getRandomNumberInInterval(-1.0, 1.0) + p[1];
		q[2] = rgen.getRandomNumberInInterval(-1.0, 1.0) + p[2];
		// Project it on the plane
		AnalyticGeometry::projectOnPlane3(q, p, n, proj);
		// Get the connecting vector
		Vector::diff(proj, p, x);
	}
	while ( Vector::length3(x) < 0.0001 );

	// Normalize 'x': this will be the x-axis
	Vector::normalize3(x);
	// Compute the y-axis
	Vector::cross3(n, x, y);

	// Save the frame
	frame[0][0] = x[0]; frame[0][1] = y[0]; frame[0][2] = n[0];
	frame[1][0] = x[1]; frame[1][1] = y[1]; frame[1][2] = n[1];
	frame[2][0] = x[2]; frame[2][1] = y[2]; frame[2][2] = n[2];
}

//===============================================================================================================================

void AnalyticGeometry::projectOnPlane3(const double* x, const double* planePoint, const double* planeNormal, double* out)
{
	double diff[3];
	// The connecting vector from 'planePoint' to 'x'
	Vector::diff(x, planePoint, diff);
	double dot = Vector::dot3(planeNormal, diff);
	double inplane[3], nproj[3] = {-dot*planeNormal[0], -dot*planeNormal[1], -dot*planeNormal[2]};
	Vector::sum3(diff, nproj, inplane);
	Vector::sum3(planePoint, inplane, out);
}

//===============================================================================================================================

void AnalyticGeometry::projectOnPlane3(const double* x, const double* planeNormal, double* out)
{
	double dot = Vector::dot3(planeNormal, x);
	// Project 'x' on the plane normal
	double nproj[3] = {-dot*planeNormal[0], -dot*planeNormal[1], -dot*planeNormal[2]};
	Vector::sum3(x, nproj, out);
}

//===============================================================================================================================
