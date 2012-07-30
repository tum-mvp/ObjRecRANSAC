#ifndef VTKMESHSAMPLER_H_
#define VTKMESHSAMPLER_H_

#include <vtkPolyData.h>
#include <vtkPoints.h>


class VtkMeshSampler
{
public:
	VtkMeshSampler();
	virtual ~VtkMeshSampler();

	/** This method samples 'numberOfPoints' points uniformly from the surface of 'in' and saves them in 'out'. */
	void sample(vtkPolyData* in, vtkPoints* out, int numberOfPoints);

	/** The method first estimates the normals for the surface 'in'. Second, it samples 'numberOfPoints' points and normals
	  * uniformly from 'in' and saves them in 'out'. */
	void estimateAndSample(vtkPolyData* in, vtkPolyData* out, int numberOfPoints);

	/** The method picks at random 'numberOfPoint' points from 'in' and saves (adds) them in 'out'. */
	void sample(vtkPoints* in, vtkPoints* out, int numberOfPoints);
};

#endif /*VTKMESHSAMPLER_H_*/
