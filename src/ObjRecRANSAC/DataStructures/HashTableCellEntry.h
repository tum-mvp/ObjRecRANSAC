#ifndef _HASH_TABLE_CELL_ENTRY_H_
#define _HASH_TABLE_CELL_ENTRY_H_

#include <vtkPolyData.h>
#include <list>

using namespace std;

class HashTableCell;
class DatabaseModelEntry;
class HashTableCellEntry;

class Key
{
public:
	Key(int pointId1, int pointId2, const double* key);
	virtual ~Key(){}

	void print(FILE* stream);

	int getPointId1()const{ return mPointId1;}
	int getPointId2()const{ return mPointId2;}

protected:
	int mPointId1, mPointId2;
	/** mKey[0] = angle between connecting line and normal 1
	  * mKey[1] = angle between connecting line and normal 2
	  * mKey[2] = angle between normals. */
	double mKey[3];
};


class HashTableCellEntry
{
public:
	HashTableCellEntry(DatabaseModelEntry* databaseModelEntry, HashTableCell* hashTableCell);
	virtual ~HashTableCellEntry();

	static bool compareEntries(const HashTableCellEntry* entry1, const HashTableCellEntry* entry2);

	DatabaseModelEntry* getDatabaseModelEntry()const{ return mDatabaseModelEntry;}
	list<Key*>& getKeys(){ return mKeys;}
	int getNumberOfKeys()const{ return mKeys.size();}

	HashTableCell* getHashTableCell(){ return mHashTableCell;}

	inline void addKey(int pointId1, int pointId2, const double *key);
	void clearKeyList();

protected:
	DatabaseModelEntry* mDatabaseModelEntry;
	list<Key*> mKeys;
	HashTableCell* mHashTableCell;
};

//=== inline methods ======================================================================================================

inline void HashTableCellEntry::addKey(int pointId1, int pointId2, const double *key)
{
	mKeys.push_back(new Key(pointId1, pointId2, key));
}

//=========================================================================================================================

#endif /*_HASH_TABLE_CELL_ENTRY_H_*/
