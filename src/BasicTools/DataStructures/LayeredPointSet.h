#ifndef LAYEREDPOINTSET_H_
#define LAYEREDPOINTSET_H_

#include <vtkPoints.h>
#include <vector>
#include <list>
#include <cstdio>

using namespace std;

//#define LPS_DEBUG

class LayeredPointSet
{
public:
	LayeredPointSet();
	virtual ~LayeredPointSet();

	void buildPointSet(vtkPoints* inputPoints, int numberOfLayers);
	void buildPointSet(int numberOfPointsPerLayer, vtkPoints* inputPoints);
	void clear();

	vtkPoints* getInputPoints()const{ return mInputPoints;}

	int getNumberOfLayers()const{ return mNumOfLayers;}
	int getNumberOfPointsInLayer(int layer)const{ return mNumOfPointsInLayer[layer];}
	const double* getLayer(int id)const{ return mLayeredPoints[id];}

	void print(FILE* stream, char* label = NULL);

protected:
	void allocLayers(int numOfLayers, int numOfPointsPerLayer, int numOfPointsInLastLayer);

protected:
	int *mNumOfPointsInLayer, mNumOfLayers;
	vtkPoints* mInputPoints;

public:
	double** mLayeredPoints;
};

#endif /*LAYEREDPOINTSET_H_*/
