#ifndef _VTK_POINTS_H_
#define _VTK_POINTS_H_

#include "VtkObject.h"
#include <vtkPoints.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkIdList.h>
#include <list>

using namespace std;


class VtkPoints: public VtkObject
{
public:
	VtkPoints(vtkIdList* ids, vtkPoints* input);
	VtkPoints(list<double*>& input);
	VtkPoints(list<const double*>& input);
	VtkPoints(list<int>& ids, vtkPoints* input);
	VtkPoints(int* ids, int numOfIds, vtkPoints* input);
	VtkPoints(double** input, int numOfPoints);
	VtkPoints(vtkPoints *input);
	VtkPoints(vtkPolyData *input, bool colorModeByScalar = false);
	virtual ~VtkPoints();

	/** Inherited from 'VtkObject' */
	vtkActor* getActor(){ return mActor;}
	/** Inherited from 'VtkObject' */
	vtkPolyData* getPolyData(){ return mPoints;}

	void updateGlyphsGeometry(){ mPoints->Update(); mGlyphs->Update(); mSphereSrc->Update();}

	double getNextPointMeanDistance();
	void setColor(double r, double g, double b){ mActor->GetProperty()->SetColor(r, g, b);}
	void visibilityOn(){ mActor->VisibilityOn();}
	void visibilityOff(){ mActor->VisibilityOff();}
	/** Self adjust the radius of the spheres representing the points in the data set. */
	void selfAdjustPointRadius();
	/** Set the radius of the spheres representing the points in the data set. */
	void setPointRadius(double radius){ mGlyphs->SetScaleFactor(radius); mGlyphs->Update();}
	double getPointRadius(){ return mGlyphs->GetScaleFactor();}

	void setResolution(int phiRes, int thetaRes){ mSphereSrc->SetPhiResolution(phiRes); mSphereSrc->SetThetaResolution(thetaRes); mGlyphs->Update();}
	void setMeanDistReductionFactor(double value){ mMeanDistReductionFactor = value;}

protected:
	void init(vtkPoints *input);

protected:
	vtkSphereSource *mSphereSrc;
	vtkGlyph3D *mGlyphs;
	vtkPolyDataMapper *mMapper;
	vtkActor *mActor;
	vtkPolyData *mPoints;
	double mMeanDistReductionFactor;
};

#endif /*_VTK_POINTS_H_*/
