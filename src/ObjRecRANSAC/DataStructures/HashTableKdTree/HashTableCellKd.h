/*
 * HashTableCellKd.h
 *
 *  Created on: Jan 11, 2010
 *      Author: papazov
 */

#ifndef _HASH_TABLE_CELL_KD_H_
#define _HASH_TABLE_CELL_KD_H_

#include <BasicTools/ComputationalGeometry/DataStructures/BSPTree/BSPTreeNodeData.h>
#include "CellEntry.h"
#include <list>

using namespace std;


class HashTableCellKd: public BSPTreeNodeData
{
public:
	HashTableCellKd();
	virtual ~HashTableCellKd();

	void addEntry(CellEntry* entry){ mEntries.push_back(entry);}
	list<CellEntry*>& getEntries(){ return mEntries;}

protected:
	list<CellEntry*> mEntries;
};

#endif /* _HASH_TABLE_CELL_KD_H_ */
