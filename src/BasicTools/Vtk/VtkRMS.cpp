/*
 * VtkRMS.cpp
 *
 *  Created on: Apr 12, 2010
 *      Author: papazov
 */

#include "VtkRMS.h"
#include <cmath>


VtkRMS::VtkRMS()
{
	mMainPolyData = NULL;
	mPointLocator = NULL;
}

VtkRMS::~VtkRMS()
{
	if ( mPointLocator )
		mPointLocator->Delete();
}

//=======================================================================================================================

void VtkRMS::setMain(vtkPolyData* main)
{
	mMainPolyData = main;

	if ( mPointLocator )
		mPointLocator->Delete();

	mPointLocator = vtkPointLocator::New();
	mPointLocator->SetDataSet(mMainPolyData);
	mPointLocator->BuildLocator();
}

//=======================================================================================================================

double VtkRMS::RMS(vtkPoints* secondary)
{
	double sum = 0.0, p[3], x[3];
	int i, n = secondary->GetNumberOfPoints();

	for ( i = 0 ; i < n ; ++i )
	{
		secondary->GetPoint(i, x);
		mMainPolyData->GetPoint(mPointLocator->FindClosestPoint(x), p);
		sum += (pow(x[0]-p[0],2.0) + pow(x[1]-p[1],2.0) + pow(x[2]-p[2],2.0));
	}

	return sqrt(sum/(double)n);
}

//=======================================================================================================================
