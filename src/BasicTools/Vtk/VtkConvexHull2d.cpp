#include "VtkConvexHull2d.h"
#include <vtkPointsProjectedHull.h>
#include <vtkCellArray.h>

VtkConvexHull2d::VtkConvexHull2d()
{
}

VtkConvexHull2d::~VtkConvexHull2d()
{
}

//================================================================================================================================

void VtkConvexHull2d::computeConvexHull(const double** points, int numOfPoints, vtkPolyData* out)
{
	vtkPointsProjectedHull* hull_obj = vtkPointsProjectedHull::New();
	  hull_obj->SetDataTypeToDouble();
	  hull_obj->SetNumberOfPoints(numOfPoints);
	double p[3];
	int i, k;

	// Insert the points in the hull
	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		p[0] = points[i][0];
		p[1] = points[i][1];
		p[2] = 0.0; // Could be that vtk changes 'p'
		hull_obj->SetPoint(i, p);
	}
	hull_obj->Update();
	// How many hull points do we have
	int numOfHullPoints = hull_obj->GetSizeCCWHullZ();
	int len = 2*numOfHullPoints;
	// Reserve memory for the hull points
	double* hullPoints = new double[len];
	// Get the hull points
	hull_obj->GetCCWHullZ(hullPoints, len);

	vtkPoints* finalPoints = vtkPoints::New();
	  finalPoints->SetDataTypeToDouble();
	  finalPoints->SetNumberOfPoints(numOfHullPoints);

	vtkCellArray* polygon = vtkCellArray::New();
	int* ids = new int[numOfHullPoints];
	for ( i = 0, k = 0 ; i < numOfHullPoints ; ++i, k += 2 )
	{
		p[0] = hullPoints[k];
		p[1] = hullPoints[k+1];
		p[2] = 0.0; // Could be that vtk changes 'p'
		finalPoints->SetPoint(i, p);
		ids[i] = i;
	}
	polygon->InsertNextCell((vtkIdType)numOfHullPoints, (vtkIdType*)ids);

	// Save the points and the cell
	out->SetPoints(finalPoints);
	out->SetPolys(polygon);

	// Clean up
	delete[] hullPoints;
	delete[] ids;
	hull_obj->Delete();
	finalPoints->Delete();
	polygon->Delete();
}

//================================================================================================================================

void VtkConvexHull2d::computeConvexHullOfProjectionOnXYPlane(vtkPoints* points, vtkPolyData* out)
{
	int numOfPoints = points->GetNumberOfPoints();
	vtkPointsProjectedHull* hull_obj = vtkPointsProjectedHull::New();
	  hull_obj->SetDataTypeToDouble();
	  hull_obj->SetNumberOfPoints(numOfPoints);
	double p[3];
	int i, k;

	// Insert the points in the hull
	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		points->GetPoint(i, p);
		hull_obj->SetPoint(i, p);
// TEST
//		p[2] = 0.0;
//		points->SetPoint(i, p);
// TEST END
	}
	hull_obj->Update();
	// How many hull points do we have
	int numOfHullPoints = hull_obj->GetSizeCCWHullZ();
	int len = 2*numOfHullPoints;
	// Reserve memory for the hull points
	double* hullPoints = new double[len];
	// Get the hull points
	hull_obj->GetCCWHullZ(hullPoints, len);

	vtkPoints* finalPoints = vtkPoints::New();
	  finalPoints->SetDataTypeToDouble();
	  finalPoints->SetNumberOfPoints(numOfHullPoints);

	vtkCellArray* polygon = vtkCellArray::New();
	int* ids = new int[numOfHullPoints];
	for ( i = 0, k = 0 ; i < numOfHullPoints ; ++i, k += 2 )
	{
		p[0] = hullPoints[k];
		p[1] = hullPoints[k+1];
		p[2] = 0.0; // Could be that vtk changes 'p'
		finalPoints->SetPoint(i, p);
		ids[i] = i;
	}
	polygon->InsertNextCell((vtkIdType)numOfHullPoints, (vtkIdType*)ids);

	// Save the points and the cell
	out->SetPoints(finalPoints);
	out->SetPolys(polygon);

	// Clean up
	delete[] hullPoints;
	delete[] ids;
	hull_obj->Delete();
	finalPoints->Delete();
	polygon->Delete();
}

//================================================================================================================================
