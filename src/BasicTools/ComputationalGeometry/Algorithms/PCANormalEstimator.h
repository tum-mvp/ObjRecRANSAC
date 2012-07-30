#ifndef PCANORMALESTIMATOR_H_
#define PCANORMALESTIMATOR_H_

#include <vtkPolyData.h>

class PCANormalEstimator
{
public:
	PCANormalEstimator();
	virtual ~PCANormalEstimator();

	/** This method should be used if no mesh info is available but just points in 'data'.
	 * Estimate the normals for 'data' based on the PCA of a neighbourhood around every point.
	 * The 'neighRadius' sets the radius of the sphere neighbourhood used for PCA. The normals
	 * are saved in 'data' (to get them use data->GetPointData()->GetNormals()).*/
	void estimateNormals(vtkPolyData* data, double neighRadius);

	/** Orients the normals in 'data' such that the angle between every normal and the connecting
	 * line from 'starPoint' to the current point is less than 90°. This means that the normals
	 * will point "outwards". Makes sense only for star shaped objects. */
	void orientNormalsForStarShape(vtkPolyData* data, double* starPoint);
};

#endif /*PCANORMALESTIMATOR_H_*/
