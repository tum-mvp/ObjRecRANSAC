/*
 * VtkRMS.h
 *
 *  Created on: Apr 12, 2010
 *      Author: papazov
 */

#ifndef VTKRMS_H_
#define VTKRMS_H_

#include <vtkPolyData.h>
#include <vtkPointLocator.h>

class VtkRMS
{
public:
	VtkRMS();
	virtual ~VtkRMS();

	void setMain(vtkPolyData* main);
	double RMS(vtkPoints* secondary);

protected:
	vtkPolyData* mMainPolyData;
	vtkPointLocator* mPointLocator;
};

#endif /* VTKRMS_H_ */
