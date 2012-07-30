/*
 * PointSetAdapter.h
 *
 *  Created on: Aug 23, 2010
 *      Author: papazov
 */

#ifndef POINTSETADAPTER_H_
#define POINTSETADAPTER_H_

#include "../DataStructures/PointSet.h"
#include <vtkPoints.h>

class PointSetAdapter
{
public:
	PointSetAdapter(){}
	virtual ~PointSetAdapter(){}

	template<class T> inline void vtk2PointSet(vtkPoints* src, PointSet<T>* dst);
	template<class T> inline void pointSet2vtk(const PointSet<T>* src, vtkPoints* dst);
};

//=== inline methods ======================================================================================================

template<class T>
inline void PointSetAdapter::vtk2PointSet(vtkPoints* src, PointSet<T>* dst)
{
	dst->alloc(src->GetNumberOfPoints());
	T bb[6], *dst_pts = dst->getPoints();
	double src_point[3];

	// Get the source point
	src->GetPoint(0, src_point);
	// Copy to the destination
	dst_pts[0] = (T)src_point[0];
	dst_pts[1] = (T)src_point[1];
	dst_pts[2] = (T)src_point[2];
	// Initialize the bounds
	bb[0] = bb[1] = dst_pts[0];
	bb[2] = bb[3] = dst_pts[1];
	bb[4] = bb[5] = dst_pts[2];

	int i;
	for ( i = 1, dst_pts += 3 ; i < src->GetNumberOfPoints() ; ++i, dst_pts += 3 )
	{
		// Get the source point
		src->GetPoint(i, src_point);
		// Copy to the destination
		dst_pts[0] = (T)src_point[0];
		dst_pts[1] = (T)src_point[1];
		dst_pts[2] = (T)src_point[2];
		// Check for the bounds
			 if ( dst_pts[0] < bb[0] ) bb[0] = dst_pts[0];
		else if ( dst_pts[0] > bb[1] ) bb[1] = dst_pts[0];
			 if ( dst_pts[1] < bb[2] ) bb[2] = dst_pts[1];
		else if ( dst_pts[1] > bb[3] ) bb[3] = dst_pts[1];
			 if ( dst_pts[2] < bb[4] ) bb[4] = dst_pts[2];
		else if ( dst_pts[2] > bb[5] ) bb[5] = dst_pts[2];
	}
	dst->setBounds(bb);
}

//=========================================================================================================================

template<class T>
inline void PointSetAdapter::pointSet2vtk(const PointSet<T>* src, vtkPoints* dst)
{
	double src_point[3];
	const T* src_pts = src->getPoints_const();
	dst->SetNumberOfPoints(src->getNumberOfPoints());

	for ( int i = 0 ; i < src->getNumberOfPoints() ; ++i, src_pts += 3 )
	{
		src_point[0] = (double)src_pts[0];
		src_point[1] = (double)src_pts[1];
		src_point[2] = (double)src_pts[2];
		dst->SetPoint(i, src_point);
	}
}

//=========================================================================================================================

#endif /* POINTSETADAPTER_H_ */
