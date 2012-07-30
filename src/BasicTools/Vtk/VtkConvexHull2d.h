#ifndef VTKCONVEXHULL2D_H_
#define VTKCONVEXHULL2D_H_

#include <vtkPolyData.h>

class VtkConvexHull2d
{
public:
	VtkConvexHull2d();
	virtual ~VtkConvexHull2d();

	/** Computes the convex hull of the 2d 'points' and inserts the points of the hull in counter clockwise order
	 * in 'out'. 'out' will contain not only the points but the convex hull polygon as well. 'points' need to be a
	 * 'numOfPoints' x 2 matrix. */
	void computeConvexHull(const double** points, int numOfPoints, vtkPolyData* out);

	/** Computes the convex hull of the 'points' projected on the xy-plane and saves the hull points and the
	 * polygon in 'out'. */
	void computeConvexHullOfProjectionOnXYPlane(vtkPoints* points, vtkPolyData* out);
};

#endif /*VTKCONVEXHULL2D_H_*/
