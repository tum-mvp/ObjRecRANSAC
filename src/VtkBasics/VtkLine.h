#ifndef VTKLINE_H_
#define VTKLINE_H_

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkTubeFilter.h>
#include <vtkLineSource.h>
#include <vtkProperty.h>

class VtkLine
{
public:
	VtkLine();
	VtkLine(double p1[3], double p2[3], double radius = 1);
	VtkLine(double x1, double y1, double z1, double x2, double y2, double z2, double radius = 1);
	virtual ~VtkLine();

	vtkActor* getActor(){ return mActor;}
	void setLength(double len);
	void setRadius(double rad){ mTube->SetRadius(rad);}
	void setColor(double r, double g, double b){ mActor->GetProperty()->SetColor(r, g, b);}

	void getPoint1(double *p){ mLine->GetPoint1(p);}
	void getPoint2(double *p){ mLine->GetPoint2(p);}

	void setPoint1(double *p){ mLine->SetPoint1(p);}
	void setPoint1(double x, double y, double z){ mLine->SetPoint1(x, y, z);}

	void setPoint2(double *p){ mLine->SetPoint2(p);}
	void setPoint2(double x, double y, double z){ mLine->SetPoint2(x, y, z);}

	vtkPolyData* getPolyData(){ return mTube->GetOutput();}

protected:
	void init(double x1, double y1, double z1, double x2, double y2, double z2, double radius);

protected:
	vtkTubeFilter *mTube;
	vtkLineSource *mLine;
	vtkPolyDataMapper *mMapper;
	vtkActor *mActor;
};

#endif /*VTKLINE_H_*/
