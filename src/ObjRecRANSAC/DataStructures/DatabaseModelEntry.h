#ifndef _DATABASE_MODEL_ENTRY_H_
#define _DATABASE_MODEL_ENTRY_H_

#include "../UserData.h"
#include "../ORRBasicTypes.h"
#include "../ORRDefines.h"
#include "ORROctree/ORROctree.h"
#include <BasicTools/LinearAlgebra/Matrix.h>
#include <BasicTools/DataStructures/PointSet.h>
#include <vtkPolyData.h>

using namespace tum;

class DatabaseModelEntry
{
public:
	DatabaseModelEntry();
	virtual ~DatabaseModelEntry();

	bool init(vtkPolyData* mesh, UserData* userData, int numberOfPointsPerLayer, int id);

	UserData* getUserData()const{ return mUserData;}
	vtkPolyData* getOwnModel(){ return mOwnModel;}
	vtkPolyData* getHighResModel(){ return mHighResModel;}
	const PointSet<double>* getOwnPointSet()const{ return &mOwnPointSet;}
	ORROctree* getModelOctree(){ return &mModelOctree;}
	int getId()const{ return mId;}

	const double* getOwnNormals()const{ return mNormals;}
	const unsigned char* getNormalsValidityArray()const{ return mValidNormal;}

	int getInstanceCounter()const{ return mInstanceCounter;}
	void incrementInstanceCounter(){ ++mInstanceCounter;}
	void resetInstanceCounter(){ mInstanceCounter = 0;}

protected:
	UserData* mUserData;
	vtkPolyData *mOwnModel, *mHighResModel;
	int mInstanceCounter, mId;
	ORROctree mModelOctree;
	PointSet<double> mOwnPointSet;
	double *mNormals;
	unsigned char *mValidNormal;
};

//=== inline methods ==========================================================================================

#endif /*_DATABASE_MODEL_ENTRY_H_*/
