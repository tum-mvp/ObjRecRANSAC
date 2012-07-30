#include "VtkPoints.h"
#include <vtkIdList.h>
#include <vtkPointLocator.h>

VtkPoints::VtkPoints(vtkIdList* ids, vtkPoints* input)
{
	double p[3];
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();

	for ( int i = 0 ; i < ids->GetNumberOfIds() ; ++i )
	{
		input->GetPoint(ids->GetId(i), p);
		points->InsertNextPoint(p);
	}

	this->init(points);
	points->Delete();
}

//===============================================================================================================================

VtkPoints::VtkPoints(list<double*>& input)
{
	vtkPoints* points = vtkPoints::New(VTK_DOUBLE);

	for ( list<double*>::iterator p = input.begin() ; p != input.end() ; ++p )
		points->InsertNextPoint(*p);

	this->init(points);
	points->Delete();
}

//===============================================================================================================================

VtkPoints::VtkPoints(list<const double*>& input)
{
	vtkPoints* points = vtkPoints::New(VTK_DOUBLE);

	for ( list<const double*>::iterator p = input.begin() ; p != input.end() ; ++p )
		points->InsertNextPoint(*p);

	this->init(points);
	points->Delete();
}

//===============================================================================================================================

VtkPoints::VtkPoints(list<int>& ids, vtkPoints* input)
{
	double p[3];
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();

	for ( list<int>::iterator it = ids.begin() ; it != ids.end() ; ++it )
	{
		input->GetPoint(*it, p);
		points->InsertNextPoint(p);
	}

	this->init(points);
	points->Delete();
}

//===============================================================================================================================

VtkPoints::VtkPoints(int* ids, int numOfIds, vtkPoints* input)
{
	double p[3];
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();

	for ( int i = 0 ; i < numOfIds ; ++i )
	{
		input->GetPoint(ids[i], p);
		points->InsertNextPoint(p);
	}

	this->init(points);
	points->Delete();
}

//===============================================================================================================================

VtkPoints::VtkPoints(double** input, int numOfPoints)
{
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();

	for ( int i = 0 ; i < numOfPoints ; ++i )
		points->InsertNextPoint(input[i]);

	this->init(points);
	points->Delete();
}

//===============================================================================================================================

VtkPoints::VtkPoints(vtkPolyData* input, bool colorModeByScalar)
{
	mMeanDistReductionFactor = 0.65;

	mSphereSrc = vtkSphereSource::New();
	mGlyphs = vtkGlyph3D::New();
	mMapper = vtkPolyDataMapper::New();
	mActor = vtkActor::New();
	mPoints = input;

	mSphereSrc->SetPhiResolution(2);
	mSphereSrc->SetThetaResolution(4);
	mSphereSrc->SetReleaseDataFlag(1);
	mSphereSrc->SetRadius(1.0);

	mGlyphs->SetScaleFactor(1.0);
	mGlyphs->SetSourceConnection(mSphereSrc->GetOutputPort());
	mGlyphs->SetInput(mPoints);

	if ( colorModeByScalar )
	{
		mGlyphs->ScalingOff();
		mGlyphs->SetColorModeToColorByScalar();
		mMapper->SetColorModeToMapScalars();
	}
	else
	{
		mGlyphs->ScalingOn();
		mMapper->ScalarVisibilityOff();
	}

	mMapper->SetInputConnection(mGlyphs->GetOutputPort());
	mActor->SetMapper(mMapper);
}

//===============================================================================================================================

VtkPoints::VtkPoints(vtkPoints *input)
{
	this->init(input);
}

//===============================================================================================================================

void VtkPoints::init(vtkPoints *input)
{
	mMeanDistReductionFactor = 0.65;

	mSphereSrc = vtkSphereSource::New();
	mGlyphs = vtkGlyph3D::New();
	mMapper = vtkPolyDataMapper::New();
	mActor = vtkActor::New();
	mPoints = vtkPolyData::New();
	  mPoints->SetPoints(input);

	mSphereSrc->SetPhiResolution(2);
	mSphereSrc->SetThetaResolution(4);
	mSphereSrc->SetReleaseDataFlag(1);
	mSphereSrc->SetRadius(1.0);

	mGlyphs->ScalingOn();
	mGlyphs->SetScaleFactor(1.0);
	mGlyphs->SetSourceConnection(mSphereSrc->GetOutputPort());
	mGlyphs->SetInput(mPoints);

	mMapper->ScalarVisibilityOff();
	mMapper->SetInputConnection(mGlyphs->GetOutputPort());

	mActor->SetMapper(mMapper);
}

//===============================================================================================================================

VtkPoints::~VtkPoints()
{
	mSphereSrc->Delete();
	mGlyphs->Delete();
	mMapper->Delete();
	mActor->Delete();
	mPoints->Delete();
}

//===============================================================================================================================

void VtkPoints::selfAdjustPointRadius()
{
	double radius = mMeanDistReductionFactor*this->getNextPointMeanDistance();

	mGlyphs->SetScaleFactor(radius);
	mGlyphs->Update();
}

//===============================================================================================================================

double VtkPoints::getNextPointMeanDistance()
{
	vtkPointLocator* locator = vtkPointLocator::New();
	  locator->SetDataSet(mPoints);
	  locator->BuildLocator();

	int i, n = mPoints->GetNumberOfPoints();
	double p1[3], p2[3], sumDist = 0.0;
	vtkIdList* ids = vtkIdList::New();

	// Get distance between the two closest points in the dataset 'polyData'.
	for ( i = 0 ; i < n ; ++i )
	{
		ids->Reset();
		mPoints->GetPoint(i, p1);
		locator->FindClosestNPoints(2, p1, ids);

		mPoints->GetPoint(ids->GetId(0), p1);
		mPoints->GetPoint(ids->GetId(1), p2);

		sumDist += sqrt(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2) + pow(p1[2]-p2[2], 2));
	}

	// Clean up
	ids->Delete();
	locator->Delete();

	return sumDist/(double)n;
}

//===============================================================================================================================
