#include "ModelDatabase.h"
#include "ORRDefines.h"
#include "DataStructures/HashTableCellEntry.h"
#include <BasicTools/Vtk/VtkTransform.h>
#include <vtkPointData.h>
#include <algorithm>

using namespace std;

ModelDatabase::ModelDatabase(double pairwidth, double voxelsize)
{
	mRelNumOfPairsToKill = 0.4;
	mVoxelSize = voxelsize;
	mPairWidth = pairwidth;
	mNormalEstimationNeighRadius = 2;
	mMaxNumOfModelPoints = 0;
	this->buildHashTable();
}

ModelDatabase::~ModelDatabase()
{
	this->clear();
}

//===================================================================================================================================

void ModelDatabase::clear()
{
	mHashTable.clear();
	for ( map<vtkPolyData*, DatabaseModelEntry*>::iterator it = mModelEntries.begin() ; it != mModelEntries.end() ; ++it )
		delete (*it).second;
	mModelEntries.clear();

#ifdef OBJ_REC_RANSAC_CUDA
	mCudaModelLibrary.clear();
#endif
}

//===================================================================================================================================

bool ModelDatabase::buildHashTable()
{
	// Setup the table bounds
	Matrix tableBounds(MDB_HASH_TABLE_DIM, 2);
	  // Bounds for the angle between the connecting line and the first normal
	  tableBounds.m[0][0] = -0.00001;
	  tableBounds.m[0][1] =  M_PI+0.00001;
	  // Bounds for the angle between the connecting line and the second normal
	  tableBounds.m[1][0] = -0.00001;
	  tableBounds.m[1][1] =  M_PI+0.00001;
	  // Bounds for the angle between both normals
	  tableBounds.m[2][0] = -0.00001;
	  tableBounds.m[2][1] =  M_PI+0.00001;

	// Setup the number of boxes in each dimension
	int numOfBoxes[] = {120, 120, 120};

	// Allocate memory for the table
	if ( !mHashTable.buildNdBoxStructure(numOfBoxes, MDB_HASH_TABLE_DIM, tableBounds.m) )
	{
		fprintf(stderr, "ERROR in 'ModelDatabase::%s()': can not build hash table.\n", __func__);
		return false;
	}

	return true;
}

//===================================================================================================================================

void ModelDatabase::resetInstanceCounters()
{
	for ( map<vtkPolyData*, DatabaseModelEntry*>::iterator it = mModelEntries.begin() ; it != mModelEntries.end() ; ++it )
		it->second->resetInstanceCounter();
}

//===================================================================================================================================

bool ModelDatabase::addModel(vtkPolyData* highResModel, UserData* userData, int numberOfPointsPerLayer, double& relNumOfPairsInHashTable)
{
	// Check if the model already exists in the database
	if ( mModelEntries.find(highResModel) != mModelEntries.end() )
	{
		fprintf(stderr, "WARNING in 'ModelDatabase::%s()': the model already exists in the database. "
				"It won't be inserted again.\n", __func__);
		fflush(stderr);
		return false;
	}

	double min[3], max[3];
	VtkTransform::getBounds(highResModel->GetPoints(), min, max);

#if defined OBJ_REC_RANSAC_VERBOSE_1 || defined OBJ_REC_RANSAC_VERBOSE_2
	printf("ModelDatabase::%s(): model bounds:\n[%8.3lf, %8.3lf]\n[%8.3lf, %8.3lf]\n[%8.3lf, %8.3lf]\n",
			__func__, min[0], max[0], min[1], max[1], min[2], max[2]); fflush(stdout);
#endif

	// Create a new model entry
	DatabaseModelEntry* dbModelEntry = new DatabaseModelEntry();
	// Build an octree for the model and estimate the normals
	if ( !mOctreeModelBuilder.buildModelOctree(highResModel, mVoxelSize, mNormalEstimationNeighRadius,
			dbModelEntry->getModelOctree(), dbModelEntry->getOwnModel()) )
	{
		fprintf(stderr, "ERROR in 'ModelDatabase::%s()': can not add the model to the database.\n", __func__);
		fflush(stderr);
		delete dbModelEntry;
		return false;
	}
	// Init the model entry
	if ( !dbModelEntry->init(highResModel, userData, numberOfPointsPerLayer, (int)mModelEntries.size()) )
	{
		fprintf(stderr, "ERROR in 'ModelDatabase::%s()': can not initialize the new model entry.\n", __func__);
		fflush(stderr);
		delete dbModelEntry;
		return false;
	}

	// Save the new model entry with 'highResModel' as key
	mModelEntries[highResModel] = dbModelEntry;

	// Check the max number of model points
	if ( dbModelEntry->getOwnModel()->GetNumberOfPoints() > mMaxNumOfModelPoints )
		mMaxNumOfModelPoints = dbModelEntry->getOwnModel()->GetNumberOfPoints();

#if defined OBJ_REC_RANSAC_VERBOSE_1 || defined OBJ_REC_RANSAC_VERBOSE_2
	printf("ModelDatabase::%s(): adding to the hash table ...\n", __func__);
	fflush(stdout);
#endif

	// Now add the model entry to the hash table (this is quite time consuming)
	relNumOfPairsInHashTable = this->addToHashTable(dbModelEntry);

#if defined OBJ_REC_RANSAC_VERBOSE_1 || defined OBJ_REC_RANSAC_VERBOSE_2
	printf("ModelDatabase::%s(): %.3lf%% of the pairs have been saved in the hash table.\n\n",
			__func__, 100.0*relNumOfPairsInHashTable);
	fflush(stdout);
#endif

	return true;
}

//===================================================================================================================================

double ModelDatabase::addToHashTable(DatabaseModelEntry* dbModelEntry)
{
	ORROctree* octree = dbModelEntry->getModelOctree();
	int i, cellId, numOfPairs = 0, numOfFullLeafs = octree->getNumberOfFullLeafs();
	vector<OctreeNode*>& fullLeafs = octree->getFullLeafs();
	ORROctreeNodeData *nodeData1, *nodeData2;
	double *n1, *n2, key[3];
	const double *p1, *p2;
	list<OctreeNode*> intLeafs;

	// Get the normals
	vector<HashTableCellEntry*> modelCellEntries;
	HashTableCell* cell;

	for ( i = 0 ; i < numOfFullLeafs ; ++i )
	{
		nodeData1 = (ORROctreeNodeData*)fullLeafs[i]->getData();
		// Get the first oriented point (point + normal)
		n1 = nodeData1->getNormal();
		if ( !n1 ) continue;
		p1 = nodeData1->getPoint();

		// Get all full leafs at the right distance to the current leaf
		intLeafs.clear();
		octree->getFullLeafsIntersectedBySphere(p1, mPairWidth, intLeafs);

		for ( list<OctreeNode*>::iterator it = intLeafs.begin() ; it != intLeafs.end() ; ++it )
		{
			nodeData2 = (ORROctreeNodeData*)((*it)->getData());
			// Get the second oriented point (point + normal)
			n2 = nodeData2->getNormal();
			if ( !n2 ) continue;
			p2 = nodeData2->getPoint();

			// Compute the hash table key
			mKeyGen.computeHashTableKey3(p1, n1, p2, n2, key);
			// Check if there is a cell with 'key' in the hash table
			cellId = mHashTable.getBoxId(key);

			// Get the cell
			bool newCellWasCreated;
			cell = mHashTable.getOrCreateCell(cellId, newCellWasCreated);

			bool newEntryWasCreated;
			// Get the cell entry which corresponds to 'dbModelEntry'
			HashTableCellEntry* entry = cell->getOrCreateCellEntry(dbModelEntry, newEntryWasCreated);
			// Save new created entries only!
			if ( newEntryWasCreated )
				modelCellEntries.push_back(entry);
			// Add the pair to the appropriate cell entry
			entry->addKey(nodeData1->getOwnPointId(), nodeData2->getOwnPointId(), key);
			++numOfPairs;
		}
	}
#ifdef MODEL_DATABASE_PRINT
	printf("ModelDatabase::%s(): %i pair(s) for the current model.\n", __func__, numOfPairs);
	fflush(stdout);
#endif

	return 1.0 - this->removePairsByKillingCellEntries(modelCellEntries, numOfPairs, mRelNumOfPairsToKill);
}

//===================================================================================================================================

double ModelDatabase::removePairsByKillingCellEntries(vector<HashTableCellEntry*>& cellEntries, int numOfPairs, double toRemove)
{
	// Sort the entries according to the number of items in each entry
	sort(cellEntries.begin(), cellEntries.end(), HashTableCellEntry::compareEntries);

	// Determine which pairs to kill
	int i, counter = 0, numOfCellEntries = (int)cellEntries.size();
	int maxNumOfPairs = (int)(((double)numOfPairs)*(1.0-toRemove) + 0.5);

	for ( i = 0 ; i < numOfCellEntries ; ++i )
	{
		counter += cellEntries[i]->getNumberOfKeys();
		if ( counter > maxNumOfPairs )
			break;
	}

	if ( i <= 0 )
	{
		fprintf(stderr, "WARNING in 'ModelDatabase::%s()': can not delete all pairs belonging to the current model!\n"
				"Choose a lower value for 'toRemove' (it has to be in [0.0, 1.0)).\n"
				"No pairs will be deleted, i.e., the recognition will work but will be slower!\n", __func__);
		return 0.0;
	}
	counter = 0;
	// Kill the cells which have too many items
	for ( ; i < numOfCellEntries ; ++i )
	{
		counter += cellEntries[i]->getNumberOfKeys();
		// Get the right cell
		HashTableCell* cell = cellEntries[i]->getHashTableCell();
		// Kill the entry from the selected cell. 'dbModelEntry' is used as key.
		cell->killCellEntry(cellEntries[i]->getDatabaseModelEntry());
		// Check if the cell is now empty (could be that there are no other model entries in the cell)
		if ( !cell->isFull() )
			mHashTable.killCell(cell);
	}

#ifdef MODEL_DATABASE_PRINT
	printf("ModelDatabase::%s(): %.3lf%% of the pairs have been removed.\n", __func__,
			100.0*(double)counter/(double)numOfPairs);
	fflush(stdout);
#endif

	return (double)counter/(double)numOfPairs;
}

//===================================================================================================================================
