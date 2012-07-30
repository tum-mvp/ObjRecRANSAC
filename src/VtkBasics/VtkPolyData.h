#ifndef VTKPOLYDATA_H_
#define VTKPOLYDATA_H_

#include "VtkObject.h"
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkLookupTable.h>
#include <cstring>


class VtkPolyData: public VtkObject
{
public:
	VtkPolyData(vtkPolyData *polyData = NULL);
	virtual ~VtkPolyData();

	/** Inherited from 'VtkObject' */
	vtkActor* getActor(){ return mActor;}

	vtkPolyData* getPolyData(){ return mPolyData;}
	vtkPoints* getPoints(){ return mPolyData->GetPoints();}
	vtkMapper* getMapper(){ return mMapper;}

	void setOpacity(double s){ mActor->GetProperty()->SetOpacity(s);}
	void setInterpolationToFlat(){ mActor->GetProperty()->SetInterpolationToFlat();}
	void setInterpolationToPhong(){ mActor->GetProperty()->SetInterpolationToPhong();}

	bool isVisible(){ return (bool)mActor->GetVisibility();}
	void visibilityOff(){ mActor->VisibilityOff();}
	void visibilityOn(){ mActor->VisibilityOn();}

	void useBlueToRedColorTable();
	inline void setLookupTable(vtkLookupTable* lut);

	void normScalars();

protected:
	vtkPolyDataMapper *mMapper;
	vtkActor *mActor;
	vtkPolyData *mPolyData;
	bool mHasOwnPolyData;
	double *mBounds;
};

//=== inline methods =======================================================================================

void VtkPolyData::setLookupTable(vtkLookupTable* lut)
{
	mMapper->SetLookupTable(lut);
	mMapper->SetColorModeToMapScalars();
	mMapper->Update();
}

//==========================================================================================================

#endif /*VTKPOLYDATA_H_*/
