#include "VtkMeshSampler.h"
#include "../Stochastics/RandomGenerator.h"
#include <vtkPolyDataNormals.h>
#include <vtkTriangleFilter.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkTriangle.h>
#include <vtkIdList.h>
#include <vtkCell.h>
#include <vector>

using namespace std;

#include "VtkTransform.h"
#include "../Stochastics/RandomGenerator.h"

VtkMeshSampler::VtkMeshSampler()
{
}

VtkMeshSampler::~VtkMeshSampler()
{
}

//============================================================================================================================

void VtkMeshSampler::sample(vtkPoints* in, vtkPoints* out, int numberOfPoints)
{
	if ( in->GetNumberOfPoints() <= numberOfPoints )
	{
		VtkTransform::copy(in, out);
		return;
	}

	double point[3];
	RandomGenerator randgen;
	int i, rand_pos, rand_id;
	vector<int> ids;
	for ( i = 0 ; i < in->GetNumberOfPoints() ; ++i )
		ids.push_back(i);

	for ( i = 0 ; i < numberOfPoints ; ++i )
	{
		// Select a random position in the id array
		rand_pos = randgen.getRandomInteger(0, ids.size()-1);
		// Get the id at that position
		rand_id = ids[rand_pos];
		// Delete the element at the selected position from the id array
		ids.erase(ids.begin() + rand_pos);

		// Get the random point
		in->GetPoint(rand_id, point);
		// Save it
		out->InsertNextPoint(point);
	}
}

//============================================================================================================================

void VtkMeshSampler::sample(vtkPolyData* in, vtkPoints* out, int numberOfPoints)
{
	vtkTriangleFilter* tria_filter = vtkTriangleFilter::New();
	  tria_filter->PassLinesOff();
	  tria_filter->PassVertsOff();
	  tria_filter->SetInput(in);
	  tria_filter->Update();
	vtkPolyData* mesh = tria_filter->GetOutput();
	  mesh->BuildLinks();
	vtkPoints* cell_pts;
	double mesh_area, area, u[3], v[3], w[3];
	vector<int> tria_ids;
	vector<double> tria_area;
	int i;

	for ( i = 0, mesh_area = 0.0 ; i < mesh->GetNumberOfCells() ; ++i )
	{
		if ( mesh->GetCellType(i) != VTK_TRIANGLE )
			continue;

		// Save the id of this triangle
		tria_ids.push_back(i);
		// Get the triangle points
		cell_pts = mesh->GetCell(i)->GetPoints();
		cell_pts->GetPoint(0, u);
		cell_pts->GetPoint(1, v);
		cell_pts->GetPoint(2, w);
		// Compute the area of the current triangle
		area = vtkTriangle::TriangleArea(u, v, w);
		// Save the area of the current triangle
		tria_area.push_back(area);
		// Increment the area of the mesh
		mesh_area += area;
	}

	int numOfTrias = (int)tria_ids.size();
	vector<double> prob_intervals;

	// Compute the probability distribution from wich the triangles will be drawn:
	for ( i = 0, area = 0.0 ; i < numOfTrias ; ++i )
	{
		area += tria_area[i];
		prob_intervals.push_back(area/mesh_area);
	}

	int id;
	RandomGenerator randgen;
	double p[3];
	// Draw a triangle according to the distribution computed above
	// and uniformly sample a point within this triangle
	for ( i = 0 ; i < numberOfPoints ; ++i )
	{
		id = randgen.drawRandomNumberFromIntervals(prob_intervals);
		// Get the triangle points
		cell_pts = mesh->GetCell(tria_ids[id])->GetPoints();
		cell_pts->GetPoint(0, u);
		cell_pts->GetPoint(1, v);
		cell_pts->GetPoint(2, w);
		// Sample a point within the selected triangle
		randgen.getRandomPointInTriangle(u, v, w, p);
		// Save the sampled point
		out->InsertNextPoint(p);
	}

	// Clean up
	tria_filter->Delete();
}

//============================================================================================================================

void VtkMeshSampler::estimateAndSample(vtkPolyData* in, vtkPolyData* out, int numberOfPoints)
{
	vtkTriangleFilter* tria_filter = vtkTriangleFilter::New();
	  tria_filter->PassLinesOff();
	  tria_filter->PassVertsOff();
	  tria_filter->SetInput(in);
	  tria_filter->Update();

	vtkPolyDataNormals* normals_filter = vtkPolyDataNormals::New();
	  normals_filter->ConsistencyOn();
	  normals_filter->ComputePointNormalsOn();
	  normals_filter->SetInputConnection(tria_filter->GetOutputPort());
	  normals_filter->Update();

	vtkPolyData* mesh = normals_filter->GetOutput();
	  mesh->BuildLinks();

	vtkPoints* cell_pts;
	double mesh_area, area, u[3], v[3], w[3];
	vector<int> tria_ids;
	vector<double> tria_area;
	int i;

	for ( i = 0, mesh_area = 0.0 ; i < mesh->GetNumberOfCells() ; ++i )
	{
		if ( mesh->GetCellType(i) != VTK_TRIANGLE )
			continue;

		// Save the id of this triangle
		tria_ids.push_back(i);
		// Get the triangle points
		cell_pts = mesh->GetCell(i)->GetPoints();
		cell_pts->GetPoint(0, u);
		cell_pts->GetPoint(1, v);
		cell_pts->GetPoint(2, w);
		// Compute the area of the current triangle
		area = vtkTriangle::TriangleArea(u, v, w);
		// Save the area of the current triangle
		tria_area.push_back(area);
		// Increment the area of the mesh
		mesh_area += area;
	}

	int numOfTrias = (int)tria_ids.size();
	vector<double> prob_intervals;

	// Compute the probability distribution from wich the triangles will be drawn:
	for ( i = 0, area = 0.0 ; i < numOfTrias ; ++i )
	{
		area += tria_area[i];
		prob_intervals.push_back(area/mesh_area);
	}

	int id;
	vtkIdList *cellPointIds;
	RandomGenerator randgen;
	double p[3], n[3];
	vtkPoints* points_out = vtkPoints::New();
	  points_out->SetDataTypeToDouble();
	vtkDataArray* normals = mesh->GetPointData()->GetNormals();
	vtkDoubleArray* normals_out = vtkDoubleArray::New();
	  normals_out->SetNumberOfComponents(3);
	  normals_out->SetNumberOfTuples(numberOfPoints);

	// Draw a triangle according to the distribution computed above
	// and uniformly sample a point within this triangle
	for ( i = 0 ; i < numberOfPoints ; ++i )
	{
		id = randgen.drawRandomNumberFromIntervals(prob_intervals);
		// Get the triangle points
		cell_pts = mesh->GetCell(tria_ids[id])->GetPoints();
		cell_pts->GetPoint(0, u);
		cell_pts->GetPoint(1, v);
		cell_pts->GetPoint(2, w);
		// Sample a point within the selected triangle
		randgen.getRandomPointInTriangle(u, v, w, p);
		// Save the sampled point
		points_out->InsertNextPoint(p);

		// Get the point ids belonging to the current triangle
		cellPointIds = mesh->GetCell(tria_ids[id])->GetPointIds();
		// Get the normals at the triangle points
		normals->GetTuple(cellPointIds->GetId(0), u);
		normals->GetTuple(cellPointIds->GetId(1), v);
		normals->GetTuple(cellPointIds->GetId(2), w);
		// Compute the final normal
		n[0] = u[0] + v[0] + w[0];
		n[1] = u[1] + v[1] + w[1];
		n[2] = u[2] + v[2] + w[2];
		Vector::normalize3(n);
		// Save the normal
		normals_out->SetTuple(i, n);
	}

	out->SetPoints(points_out);
	out->GetPointData()->SetNormals(normals_out);

	// Cleanup
	tria_filter->Delete();
	normals_filter->Delete();
	points_out->Delete();
	normals_out->Delete();
}

//============================================================================================================================
