/*
 * CellEntry.cpp
 *
 *  Created on: Jan 11, 2010
 *      Author: papazov
 */

#include "CellEntry.h"

CellEntry::CellEntry(DatabaseModelEntry *dbModelEntry, double *point)
{
	mPoint = point;
	mDBModelEntry = dbModelEntry;
}

CellEntry::~CellEntry()
{
//	if ( mPoint )
//		delete[] mPoint;
}
