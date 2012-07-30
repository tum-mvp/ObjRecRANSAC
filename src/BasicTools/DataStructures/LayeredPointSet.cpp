#include "LayeredPointSet.h"
#include "../Stochastics/RandomGenerator.h"


LayeredPointSet::LayeredPointSet()
{
	mNumOfLayers = 0;
	mNumOfPointsInLayer = NULL;
	mLayeredPoints = NULL;
}

LayeredPointSet::~LayeredPointSet()
{
	this->clear();
}

//================================================================================================================================

void LayeredPointSet::clear()
{
	if ( mNumOfPointsInLayer )
	{
		delete[] mNumOfPointsInLayer;
		mNumOfPointsInLayer = NULL;
	}

	if ( mLayeredPoints )
	{
		for ( int i = 0 ; i < mNumOfLayers ; ++i )
			delete[] mLayeredPoints[i];
		mLayeredPoints = NULL;
	}

	mNumOfLayers = 0;
}

//================================================================================================================================

void LayeredPointSet::buildPointSet(int numberOfPointsPerLayer, vtkPoints* inputPoints)
{
	mInputPoints = inputPoints;

	int numOfLayers = (int)((double)inputPoints->GetNumberOfPoints()/(double)numberOfPointsPerLayer + 0.5);
	if ( numOfLayers == 0 )
		numOfLayers = 1;

	this->buildPointSet(inputPoints, numOfLayers);
}

//================================================================================================================================

void LayeredPointSet::buildPointSet(vtkPoints* inputPoints, int numberOfLayers)
{
	mInputPoints = inputPoints;

	int i, id, numOfInputPoints = inputPoints->GetNumberOfPoints();
	double p[3];

	this->clear();

	int layer, randint, numOfPointsPerLayer = numOfInputPoints/numberOfLayers, 
	remainder = numOfInputPoints % numberOfLayers;

	if ( numOfPointsPerLayer == 0 )
	{
		remainder = 0;
		numOfPointsPerLayer = 1;
		numberOfLayers = numOfInputPoints;
	}

	// Allocate memory for the point layers
	this->allocLayers(numberOfLayers, numOfPointsPerLayer, numOfPointsPerLayer + remainder);

	RandomGenerator randgen;
	vector<int> allids;
	  allids.reserve(numOfInputPoints);
	// Init the vector with all ids
	for ( i = 0 ; i < numOfInputPoints ; ++i )
		allids.push_back(i);

	// For all but the last layer
	for ( layer = 0 ; layer < numberOfLayers-1 ; ++layer )
	{
		// Fill the layer with random points
		for ( i = 0 ; i < numOfPointsPerLayer ; ++i )
		{
			// Get an id at random (from the remaining ids)
			randint = randgen.getRandomInteger(0, allids.size()-1);
			// Get the randomly selected point
			inputPoints->GetPoint(allids[randint], p);
			// Save the point in the own point array
			id = 3*i;
			mLayeredPoints[layer][id]   = p[0];
			mLayeredPoints[layer][id+1] = p[1];
			mLayeredPoints[layer][id+2] = p[2];
			// Erase the selected id
			allids.erase(allids.begin() + randint);
		}
	}

	if ( (int)allids.size() != mNumOfPointsInLayer[layer] )
	{
		fprintf(stderr, "ERROR in 'LayeredPointSet::%s()': number of points in the last layer is wrong!\n", __func__);
		fflush(stderr);
		return;
	}

	// Create the last layer and fill it with the remaining ids
	numOfPointsPerLayer = allids.size();
	for ( i = 0 ; i < numOfPointsPerLayer ; ++i )
	{
		inputPoints->GetPoint(allids[i], p);
		id = 3*i;
		mLayeredPoints[layer][id]   = p[0];
		mLayeredPoints[layer][id+1] = p[1];
		mLayeredPoints[layer][id+2] = p[2];
	}
}

//================================================================================================================================

void LayeredPointSet::allocLayers(int numOfLayers, int numOfPointsPerLayer, int numOfPointsInLastLayer)
{
	mNumOfLayers = numOfLayers;
	mNumOfPointsInLayer = new int[mNumOfLayers];
	mLayeredPoints = new double*[mNumOfLayers];

	int i;
	for ( i = 0 ; i < numOfLayers-1 ; ++i )
	{
		mNumOfPointsInLayer[i] = numOfPointsPerLayer;
		mLayeredPoints[i] = new double[3*numOfPointsPerLayer];
	}
	mNumOfPointsInLayer[i] = numOfPointsInLastLayer;
	mLayeredPoints[i] = new double[3*numOfPointsInLastLayer];
}

//================================================================================================================================

void LayeredPointSet::print(FILE* stream, char* label)
{
	if ( label ) fprintf(stream, "---------------- %s ----------------\n", label);
	else fprintf(stream, "------------------------- Layered Point Set Info -----------------------------\n");

	fprintf(stream, "Number of layers: %i\nNumber of points per layer: [", mNumOfLayers);
	for ( int i = 0 ; i < mNumOfLayers ; ++i )
		fprintf(stream, " %i ", mNumOfPointsInLayer[i]);
	fprintf(stream, "]\n");

#ifdef LPS_DEBUG
	for ( int i = 0 ; i < mNumOfLayers ; ++i )
	{
		fprintf(stream, "Layer %i: ", i);
		for ( int j = 0 ; j < mNumOfPointsInLayer[i] ; ++j )
			fprintf(stream, " %i ", mPointLayers[i][j]);
		fprintf(stream, "\n");
	}
#endif

	fprintf(stream, "------------------------------------------------------------------------------\n");
	fflush(stream);
}

//================================================================================================================================
