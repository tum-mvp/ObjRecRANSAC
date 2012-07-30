/*
 * GraphNode.h
 *
 *  Created on: Jan 22, 2010
 *      Author: papazov
 */

#ifndef _TUM_GRAPHNODE_H_
#define _TUM_GRAPHNODE_H_

#include "GraphNodeData.h"
#include <map>

using namespace std;

#define TGraphNode GraphNode<_Key_,_Compare_>
#define GN_MAX_LABEL_LEN	512

namespace tum
{

template <class _Key_, class _Compare_>
class GraphNode
{
public:
	GraphNode(_Key_ key);
	virtual ~GraphNode();

	void setData(GraphNodeData* data){ mData = data;}
	GraphNodeData* getData(){ return mData;}
	_Key_ getKey(){ return mKey;}
	int getNumberOfNeighbours(){ return mNeighs.size();}

	inline pair<typename map<_Key_,TGraphNode*>::iterator,bool> addNeighbour(TGraphNode* neigh);

	map<_Key_,GraphNode<_Key_,_Compare_>*,_Compare_>& getNeighbours(){ return mNeighs;}

	void setTmpId(int value){ mTmpId = value;}
	int getTmpId(){ return mTmpId;}

protected:
	map<_Key_, GraphNode<_Key_,_Compare_>*, _Compare_> mNeighs;
	GraphNodeData* mData;
	_Key_ mKey;
	int mTmpId;
};

}//namespace tum

//=== inline methods ========================================================================================================================

template <class _Key_, class _Compare_>
tum::TGraphNode::GraphNode(_Key_ key)
{
	mKey = key;
	mData = NULL;
}

//===========================================================================================================================================

template <class _Key_, class _Compare_>
tum::TGraphNode::~GraphNode()
{
	if ( mData ) delete mData;
}

//===========================================================================================================================================

template <class _Key_, class _Compare_>
inline pair<typename map<_Key_,tum::TGraphNode*>::iterator,bool> tum::TGraphNode::addNeighbour(TGraphNode* neigh)
{
	return mNeighs.insert(pair<_Key_,TGraphNode*>(neigh->getKey(), neigh));
}

//===========================================================================================================================================

#undef TGraphNode

#endif /* _TUM_GRAPHNODE_H_ */
