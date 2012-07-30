#include "HashTableCell.h"

HashTableCell::HashTableCell(int cellId)
{
	mCellId = cellId;
}

HashTableCell::~HashTableCell()
{
	map<DatabaseModelEntry*, HashTableCellEntry*>::iterator it;
	// Delete all objects of type 'HashTableModelEntry'
	for ( it = mCellEntries.begin() ; it != mCellEntries.end() ; ++it )
		delete (*it).second;
	mCellEntries.clear();
}

//==============================================================================================================================
