/*
 * GraphNodeData.cpp
 *
 *  Created on: Jan 22, 2010
 *      Author: papazov
 */

#include "GraphNodeData.h"

using namespace tum;

GraphNodeData::GraphNodeData(const char* label)
{
	if ( label )
		this->setLabel(label);
	else
		mLabel[0] = '\0';
}

GraphNodeData::~GraphNodeData()
{
}
