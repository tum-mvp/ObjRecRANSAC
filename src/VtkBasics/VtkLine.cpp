#include "VtkLine.h"

VtkLine::VtkLine()
{
	this->init(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.2);
}

VtkLine::VtkLine(double p1[3], double p2[3], double radius)
{
	this->init(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], radius);
}

VtkLine::VtkLine(double x1, double y1, double z1, double x2, double y2, double z2, double radius)
{
	this->init(x1, y1, z1, x2, y2, z2, radius);
}

void VtkLine::init(double x1, double y1, double z1, double x2, double y2, double z2, double radius)
{
	mLine = vtkLineSource::New();
	mTube = vtkTubeFilter::New();
	mMapper = vtkPolyDataMapper::New();
	mActor = vtkActor::New();

	mLine->SetPoint1(x1, y1, z1);
	mLine->SetPoint2(x2, y2, z2);
	mTube->SetInput(mLine->GetOutput());
	mTube->SetNumberOfSides(8);
	mTube->SetRadius(radius);
	mMapper->SetInput(mTube->GetOutput());
	mActor->SetMapper(mMapper);
	mActor->GetProperty()->SetColor(1.0, 0.0, 0.0);
	mActor->SetPosition(0.0, 0.0, 0.0);
}

VtkLine::~VtkLine()
{
	mLine->Delete();
	mTube->Delete();
	mMapper->Delete();
	mActor->Delete();
}

//==============================================================================================================================

void VtkLine::setLength(double len)
{
	double p1[3], p2[3];

	mLine->GetPoint1(p1);
	mLine->GetPoint2(p2);

	double v[3];
	v[0] = p2[0]-p1[0];
	v[1] = p2[1]-p1[1];
	v[2] = p2[2]-p1[2];

	// Length of v
	double vlen = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

	if ( vlen < 1e-18 )
		return;

	double q = len/vlen;

	// Set v to the new length
	v[0] *= q;
	v[1] *= q;
	v[2] *= q;
	// Compute the new p2
	p2[0] = p1[0]+v[0];
	p2[1] = p1[1]+v[1];
	p2[2] = p1[2]+v[2];
	mLine->SetPoint2(p2);
}

//==============================================================================================================================
