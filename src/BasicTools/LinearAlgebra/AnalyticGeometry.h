#ifndef _TUM_ANALYTICGEOMETRY_H_
#define _TUM_ANALYTICGEOMETRY_H_

namespace tum
{

class AnalyticGeometry
{
public:
	AnalyticGeometry();
	virtual ~AnalyticGeometry();

	/** Computes the intersection point between the line through 'a' and 'b' and the line through 'c' and 'd'
	 * and saves it in 'x'. If both lines do not intersect, the method computes the point nearest to both lines.
	 * The method computes 'r1' and 'r2' which have the following meaning: x = a + r1*(b - a) = c + r2*(d - c) if
	 * the lines intersect and
	 * a + r1*(b - a) = x1, where x1 is the projection of x onto the first line and
	 * c + r2*(d - c) = x2, where x2 is the projection of x onto the second line if the lines do not intersect.
	 * 
	 * The method saves in 'dist' the distance between 'x' and the lines (zero if the lines intersect).
	 * 
	 * The result is undefined if the lines are parallel or some of them are not lines (i.e., two identical points)
	 * and the method returns 'false'.
	 *  */
	static bool getIntersectionPoint(const double* a, const double* b, const double* c, const double* d,
			double& r1, double& r2, double& dist, double* x);

	/** Returns the signed distance from 'p' to the plane defined by the three points 'a', 'b' and 'c'.
	  * The sign is positive if 'p' is on the side of (b-a) x (c-a). */
	static double signedDistanceToPlane3(const double a[3], const double b[3], const double c[3], const double p[3]);

	/** Projects 'x' on the plane defined by 'planePoint' and 'planeNormal' and saves the result in 'out'.
	  * All arrays are assumed to have enough space for three doubles. */
	static void projectOnPlane3(const double* x, const double* planePoint, const double* planeNormal, double* out);
	/** Projects 'x' on the plane through 0 and with normal 'planeNormal' and saves the result in 'out'.
	  * All arrays are assumed to have enough space for three doubles. */
	static void projectOnPlane3(const double* x, const double* planeNormal, double* out);

	/** Computes a right handed coordinate frame for the plane defined by a plane point 'p' and the plane normal 'n'
	 * such that the z axis coincides with 'n'. The x and y axes are oriented randomly but, of course, lie in the
	 * plane and are orthogonal to each other. */
	static void computeOrthonormalFrameForPlane3(const double* p, const double* n, double** frame);

	static void transformToLocalCoordinates();
};

}//namespace tum

#endif /*_TUM_ANALYTICGEOMETRY_H_*/
