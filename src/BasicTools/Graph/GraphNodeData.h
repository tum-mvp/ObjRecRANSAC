/*
 * GraphNodeData.h
 *
 *  Created on: Jan 22, 2010
 *      Author: papazov
 */

#ifndef _TUM_GRAPHNODEDATA_H_
#define _TUM_GRAPHNODEDATA_H_

#include <cstdio>

#define GND_MAX_LABEL_SIZE 512

namespace tum
{

class GraphNodeData
{
public:
	GraphNodeData(const char* label = NULL);
	virtual ~GraphNodeData();

	void setLabel(const char* label){ sprintf(mLabel, "%s", label);}
	char* getLabel(){ return mLabel;}

protected:
	char mLabel[GND_MAX_LABEL_SIZE];
};

}//namespace tum

#endif /* _TUM_GRAPHNODEDATA_H_ */
