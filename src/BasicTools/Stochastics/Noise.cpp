#include "Noise.h"
#include <vtkMath.h>
#include <ctime>

using namespace tum;

Noise::Noise()
{
}

Noise::~Noise()
{
}

//===========================================================================================================================

void Noise::corruptZ(vtkPoints* points, double min, double max)
{
	double p[3];
	for ( int i = 0 ; i < points->GetNumberOfPoints() ; ++i )
	{
		points->GetPoint(i, p);
		p[2] += mRandGenerator.getRandomNumberInInterval(min, max);
		points->SetPoint(i, p);
	}
}

//===========================================================================================================================

void Noise::corruptXYZ(vtkPoints* points, double xbounds[2], double ybounds[2], double zbounds[2])
{
	double p[3];
	for ( int i = 0 ; i < points->GetNumberOfPoints() ; ++i )
	{
		points->GetPoint(i, p);
		p[0] += mRandGenerator.getRandomNumberInInterval(xbounds[0], xbounds[1]);
		p[1] += mRandGenerator.getRandomNumberInInterval(ybounds[0], ybounds[1]);
		p[2] += mRandGenerator.getRandomNumberInInterval(zbounds[0], zbounds[1]);
		points->SetPoint(i, p);
	}
}

//===========================================================================================================================

void Noise::addGaussianNoise(vtkPoints* points, double sigmaAsPercentFromBBDiag)
{
	double bounds[6], p[3];
	points->ComputeBounds();
	points->GetBounds(bounds);
	double sigma = (sigmaAsPercentFromBBDiag/100.0)*sqrt( pow(bounds[1]-bounds[0],2) + pow(bounds[3]-bounds[2],2) + pow(bounds[5]-bounds[4],2) );

	sigma = sqrt(sigma);

	for ( int i = 0 ; i < points->GetNumberOfPoints() ; ++i )
	{
		points->GetPoint(i, p);
		p[0] += sigma*this->getGaussianSample();
		p[1] += sigma*this->getGaussianSample();
		p[2] += sigma*this->getGaussianSample();
		points->SetPoint(i, p);
	}
}

//===========================================================================================================================

void Noise::addOutliers(vtkPoints* points, int numOfOutliers)
{
	double p[3], bounds[6];
	this->getBounds(points, bounds);

	for ( int i = 0 ; i < numOfOutliers ; ++i )
	{
		mRandGenerator.getRandom3dPointWithinBox(bounds, p);
		points->InsertNextPoint(p);
	}
}

//===========================================================================================================================
