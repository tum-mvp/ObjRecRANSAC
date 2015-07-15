#ifndef MODELDATABASE_H_
#define MODELDATABASE_H_

#include "ORRDefines.h"
#include "DataStructures/HashTableBoxStruct/HashTableBoxStruct.h"
#include "Algorithms/HashTableKeyGenerator.h"
#include "Algorithms/OctreeModelBuilder.h"
#include "DataStructures/DatabaseModelEntry.h"
#include "UserData.h"
#include <vtkPolyData.h>
#include <map>

using namespace std;

#define MDB_HASH_TABLE_DIM  3

class ModelDatabase
{
public:
	ModelDatabase(double pairwidth, double voxelsize);
	virtual ~ModelDatabase();

	bool addModel(vtkPolyData* highResModel, UserData* userData, int numberOfPointsPerLayer, double& relNumOfPairsInHashTable);

	/** 'val' has to be in the open interval (0, 1). */
	void setRelativeNumberOfPairsToKill(double val){ mRelNumOfPairsToKill = val;}

	const HashTableBoxStruct* getHashTable()const{ return &mHashTable;}
	inline DatabaseModelEntry* getModelEntry(vtkPolyData* polydata);
	int getNumberOfModels(){ return (int)mModelEntries.size();}
	int getMaxNumberOfModelPoints(){ return mMaxNumOfModelPoints;}

	map<vtkPolyData*, DatabaseModelEntry*>::const_iterator getModelEntriesBegin(){ return mModelEntries.begin();}
	map<vtkPolyData*, DatabaseModelEntry*>::const_iterator getModelEntriesEnd(){ return mModelEntries.end();}

	const map<vtkPolyData*, DatabaseModelEntry*>& getModelEntries()const{ return mModelEntries;}

	void resetInstanceCounters();

	/** Clears the data base. */
	void clear();

protected:
	bool buildHashTable();
	double addToHashTable(DatabaseModelEntry* dbModelEntry);
	/** This method removes N% of the pairs by killing the most full cell entries in 'cellEntries',
	 * where N = 100*'toRemove', i.e., 'toRemove' has to be a number in the open interval (0, 1).
	 * All entries in 'cellEntries' should belong to the same model. The method returns the actual
	 * (relative) number of removed pairs. */
	double removePairsByKillingCellEntries(vector<HashTableCellEntry*>& cellEntries, int numOfPairs, double toRemove);

protected:
	HashTableBoxStruct mHashTable;
	map<vtkPolyData*, DatabaseModelEntry*> mModelEntries;
	HashTableKeyGenerator mKeyGen;
	double mVoxelSize, mPairWidth, mRelNumOfPairsToKill;
	OctreeModelBuilder mOctreeModelBuilder;
	int mNormalEstimationNeighRadius, mMaxNumOfModelPoints;
#ifdef OBJ_REC_RANSAC_CUDA
	CudaModelLibrary mCudaModelLibrary;
#endif

	// To be integrated (makes sense for high dimensional hash tables)
//	HashTableKdTree* mHashTableKd;
};

//=== inline methods ===================================================================================================

inline DatabaseModelEntry* ModelDatabase::getModelEntry(vtkPolyData* polydata)
{
	map<vtkPolyData*, DatabaseModelEntry*>::iterator it = mModelEntries.find(polydata);
	if ( it == mModelEntries.end() )
		return NULL;

	return it->second;
}

//======================================================================================================================

#endif /*MODELDATABASE_H_*/
