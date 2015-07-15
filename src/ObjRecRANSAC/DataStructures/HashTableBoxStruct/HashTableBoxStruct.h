#ifndef _HASHTABLE_BOXSTRUCT_H_
#define _HASHTABLE_BOXSTRUCT_H_

#include <BasicTools/DataStructures/NdBoxStructure.h>
#include "HashTableCell.h"
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vector>
#include <set>

using namespace std;


class HashTableBoxStruct: public NdBoxStructure<HashTableCell>
{
public:
	HashTableBoxStruct();
	virtual ~HashTableBoxStruct();

	void clear();

	/** This method creates and returns a pointer to a new cell and sets its id to 'cellId'.
	 * If there already exists a cell at position 'cellId' the method gives back a pointer
	 * to it. */
	inline HashTableCell* getOrCreateCell(int cellId, bool& newCellwasCreated);
	set<HashTableCell*>& getFullCells(){ return mFullCells;}

	inline void killCell(int id);
	inline void killCell(HashTableCell* cell);

protected:
	set<HashTableCell*> mFullCells;
};

//=== inline methods =========================================================================================================

inline HashTableCell* HashTableBoxStruct::getOrCreateCell(int cellId, bool& newCellWasCreated)
{
	if ( !this->mData[cellId] )
	{
		mData[cellId] = new HashTableCell(cellId);
		newCellWasCreated = true;
		// Save the created cell in the set of full cells
		mFullCells.insert(mData[cellId]);
	}

	newCellWasCreated = false;
	return mData[cellId];
}

//============================================================================================================================

inline void HashTableBoxStruct::killCell(int id)
{
	if ( !mData[id] )
		return;

	mFullCells.erase(mData[id]);
	delete mData[id];
	mData[id] = NULL;
}

//============================================================================================================================

inline void HashTableBoxStruct::killCell(HashTableCell* cell)
{
	int id = cell->getCellId();
	mFullCells.erase(cell);
	delete cell;
	mData[id] = NULL;
}

//============================================================================================================================

#endif /*_HASHTABLE_BOXSTRUCT_H_*/
