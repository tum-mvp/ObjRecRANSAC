/*
 * CellEntry.h
 *
 *  Created on: Jan 11, 2010
 *      Author: papazov
 */

#ifndef _CELL_ENTRY_H_
#define _CELL_ENTRY_H_

#include "../DatabaseModelEntry.h"

class CellEntry
{
public:
	CellEntry(DatabaseModelEntry *dbModelEntry, double *point);
	virtual ~CellEntry();

public:
	double *mPoint;
	DatabaseModelEntry *mDBModelEntry;
};

#endif /*_CELL_ENTRY_H_*/
