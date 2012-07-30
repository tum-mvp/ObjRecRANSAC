#include "HashTableBoxStruct.h"
#include <algorithm>

//============================================================================================================================

HashTableBoxStruct::HashTableBoxStruct()
{
}

HashTableBoxStruct::~HashTableBoxStruct()
{
	this->clear();
}

//============================================================================================================================

void HashTableBoxStruct::clear()
{
	NdBoxStructure<HashTableCell>::clear();
	mFullCells.clear();
}

//============================================================================================================================
