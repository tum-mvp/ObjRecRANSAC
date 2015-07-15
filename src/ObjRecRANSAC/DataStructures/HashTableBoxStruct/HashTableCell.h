#ifndef _HASHTABLE_CELL_H_
#define _HASHTABLE_CELL_H_

#include "../HashTableCellEntry.h"
#include <vtkPolyData.h>
#include <cstdio>
#include <list>
#include <map>

using namespace std;

class DatabaseModelEntry;

/** The class represents a cell in a hash table. Each entry in this cell corresponds to a different model (more
 * precisely to a different database model entry). */
class HashTableCell
{
public:
	HashTableCell(int cellId);
	virtual ~HashTableCell();

	/** This method returns a pointer to a new created cell entry with key 'dbModelEntry' or to an existing one.
	 * 'newEntryWasCreated' is set to true if a new entry was created, i.e., if there was no entry
	 * with key 'dbModelEntry' before calling this method. Otherwise the method sets 'newEntryWasCreated'
	 * to 'false'. */
	inline HashTableCellEntry* getOrCreateCellEntry(DatabaseModelEntry* dbModelEntry, bool& newEntryWasCreated);

	inline void killCellEntry(DatabaseModelEntry* databaseModelEntry);
	inline int getNumberOfKeys(DatabaseModelEntry* dbModelEntry);

	int getCellId(){ return mCellId;}
	const map<DatabaseModelEntry*, HashTableCellEntry*>& getCellEntries()const{ return mCellEntries;}
	bool isFull(){ return (bool)mCellEntries.size();}

protected:
	map<DatabaseModelEntry*, HashTableCellEntry*> mCellEntries;
	int mCellId;
};

//=== inline methods ===========================================================================================================

inline HashTableCellEntry* HashTableCell::getOrCreateCellEntry(DatabaseModelEntry* dbModelEntry, bool& newEntryWasCreated)
{
	// Try to find an entry with key 'model'
	map<DatabaseModelEntry*, HashTableCellEntry*>::iterator it = mCellEntries.find(dbModelEntry);
	// Check if we found something
	if ( it == mCellEntries.end() )
	{
		// Create a new model entry
		HashTableCellEntry* new_entry = new HashTableCellEntry(dbModelEntry, this);
		pair<DatabaseModelEntry*, HashTableCellEntry*> new_pair(dbModelEntry, new_entry);
		// Insert the new pair in the map
		mCellEntries.insert(new_pair);
		newEntryWasCreated = true;
		// Return the pointer to the new entry
		return new_entry;
	}
	newEntryWasCreated = false;
	return (*it).second;
}

//==============================================================================================================================

inline void HashTableCell::killCellEntry(DatabaseModelEntry* databaseModelEntry)
{
	map<DatabaseModelEntry*, HashTableCellEntry*>::iterator it = mCellEntries.find(databaseModelEntry);
	if ( it == mCellEntries.end() )
		return;

	delete it->second;
	mCellEntries.erase(it);
}

//==============================================================================================================================

inline int HashTableCell::getNumberOfKeys(DatabaseModelEntry* dbModelEntry)
{
	map<DatabaseModelEntry*, HashTableCellEntry*>::iterator it = mCellEntries.find(dbModelEntry);
	if ( it == mCellEntries.end() )
		return 0;
	return it->second->getNumberOfKeys();
}

//==============================================================================================================================

#endif /*_HASHTABLE_CELL_H_*/
