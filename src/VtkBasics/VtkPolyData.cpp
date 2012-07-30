#include "VtkPolyData.h"
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>


VtkPolyData::VtkPolyData(vtkPolyData *polyData)
{
	if ( polyData )
	{
		mPolyData = polyData;
		mHasOwnPolyData = false;
	}
	else
	{
		mPolyData = vtkPolyData::New();
		mHasOwnPolyData = true;
	}

	mMapper = vtkPolyDataMapper::New();
	mMapper->SetInput(mPolyData);

	mActor = vtkActor::New();
	mActor->SetMapper(mMapper);

	this->setInterpolationToFlat();

	sprintf(mName, "VtkPolyData-object");
	mBounds = NULL;
}

VtkPolyData::~VtkPolyData()
{
	if ( mBounds )
		delete[] mBounds;
	if ( mHasOwnPolyData )
		mPolyData->Delete();
	mActor->Delete();
	mMapper->Delete();
}

//=====================================================================================================================

void VtkPolyData::normScalars()
{
	vtkDoubleArray* newScalars = vtkDoubleArray::New();
	  newScalars->SetNumberOfTuples(1);
	vtkDataArray* oldScalars = mPolyData->GetPointData()->GetScalars();

	double range[2]; oldScalars->GetRange(range);

	printf("range = [%lf, %lf]\n", range[0], range[1]); fflush(stdout);

	for ( int i = 0 ; i < oldScalars->GetNumberOfTuples() ; ++i )
		newScalars->InsertNextTuple1((oldScalars->GetTuple1(i) - range[0])/(range[1]-range[0]));

	mPolyData->GetPointData()->SetScalars(newScalars);
	// Clean up
	newScalars->Delete();
}

//=====================================================================================================================

void VtkPolyData::useBlueToRedColorTable()
{
	vtkLookupTable* lut = vtkLookupTable::New();
	  lut->SetHueRange(0.6667, 0.0);
	  lut->SetSaturationRange(1.0, 1.0);
	  lut->SetValueRange(1.0, 1.0);
	  lut->SetAlphaRange(1.0, 1.0);
	  lut->SetNumberOfColors(256);
	  lut->Build();
	mMapper->SetLookupTable(lut);
	lut->Delete();
	mMapper->SetColorModeToMapScalars();
	mMapper->Update();
}

//=====================================================================================================================
