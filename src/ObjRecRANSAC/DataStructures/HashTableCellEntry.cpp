#include "HashTableCellEntry.h"

bool HashTableCellEntry::compareEntries(const HashTableCellEntry* entry1, const HashTableCellEntry* entry2)
{
	return (bool)(entry1->getNumberOfKeys() < entry2->getNumberOfKeys());
}

//=============================================================================================================================

HashTableCellEntry::HashTableCellEntry(DatabaseModelEntry* databaseModelEntry, HashTableCell* hashTableCell)
{
	mDatabaseModelEntry = databaseModelEntry;
	mHashTableCell = hashTableCell;
}

HashTableCellEntry::~HashTableCellEntry()
{
	this->clearKeyList();
}

//==============================================================================================================================

void HashTableCellEntry::clearKeyList()
{
	for ( list<Key*>::iterator it = mKeys.begin() ; it != mKeys.end() ; ++it )
		delete *it;
	mKeys.clear();
}

//==============================================================================================================================

Key::Key(int pointId1, int pointId2, const double* key)
{
	mPointId1 = pointId1;
	mPointId2 = pointId2;
	mKey[0] = key[0];
	mKey[1] = key[1];
	mKey[2] = key[2];
}

//==============================================================================================================================

void Key::print(FILE* stream)
{
	fprintf(stream, "(%i, %i): angles = (%lf, %lf, %lf)\n",
			mPointId1, mPointId2, mKey[0], mKey[1], mKey[2]);
	fflush(stream);
}

//==============================================================================================================================
