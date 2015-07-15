/*
 * HashTableCellKd.cpp
 *
 *  Created on: Jan 11, 2010
 *      Author: papazov
 */

#include "HashTableCellKd.h"

HashTableCellKd::HashTableCellKd()
{
}

HashTableCellKd::~HashTableCellKd()
{
	list<CellEntry*>::iterator it;
	for ( it = mEntries.begin() ; it != mEntries.end() ; ++it )
		delete *it;
	mEntries.clear();
}
