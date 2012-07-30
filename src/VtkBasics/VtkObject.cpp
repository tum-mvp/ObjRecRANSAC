#include "VtkObject.h"

VtkObject::VtkObject()
{
	mBoundsAreComputed = false;
}

VtkObject::~VtkObject()
{
}

//===================================================================================================================================

void VtkObject::setName(const char *name)
{
	int numOfBytes = strlen(name);
	if ( numOfBytes >= VTK_OBJ_MAX_NAME_LEN )
		numOfBytes = VTK_OBJ_MAX_NAME_LEN-1;

	memcpy(mName, name, numOfBytes);
	mName[numOfBytes] = '\0';
}

//===================================================================================================================================

void VtkObject::computeBounds()
{
	double p[3];
	vtkPolyData* polydata = this->getPolyData();
	// Init
	polydata->GetPoint(0, p);
	mMinBound[0] = mMaxBound[0] = p[0];
	mMinBound[1] = mMaxBound[1] = p[1];
	mMinBound[2] = mMaxBound[2] = p[2];

	for ( int i = 1 ; i < polydata->GetNumberOfPoints() ; ++i )
	{
		polydata->GetPoint(i, p);

		if ( p[0] < mMinBound[0] )
			mMinBound[0] = p[0];
		else if ( p[0] > mMaxBound[0] )
			mMaxBound[0] = p[0];

		if ( p[1] < mMinBound[1] )
			mMinBound[1] = p[1];
		else if ( p[1] > mMaxBound[1] )
			mMaxBound[1] = p[1];

		if ( p[2] < mMinBound[2] )
			mMinBound[2] = p[2];
		else if ( p[2] > mMaxBound[2] )
			mMaxBound[2] = p[2];
	}
	mBoundsAreComputed = true;
}

//===================================================================================================================================

void VtkObject::getBounds(double min[3], double max[3])
{
	if ( !mBoundsAreComputed )
		this->computeBounds();

	min[0] = mMinBound[0];
	min[1] = mMinBound[1];
	min[2] = mMinBound[2];
	max[0] = mMaxBound[0];
	max[1] = mMaxBound[1];
	max[2] = mMaxBound[2];
}

//=====================================================================================================================

void VtkObject::getCenterOfMass(double com[3])
{
	vtkPolyData *polydata = this->getPolyData();
	int i, numOfPoints = polydata->GetNumberOfPoints();
	double p[3];
	com[0] = com[1] = com[2] = 0.0;

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		polydata->GetPoint(i, p);
		com[0] += p[0];
		com[1] += p[1];
		com[2] += p[2];
	}

	com[0] /= (double)numOfPoints;
	com[1] /= (double)numOfPoints;
	com[2] /= (double)numOfPoints;
}

//===================================================================================================================================
