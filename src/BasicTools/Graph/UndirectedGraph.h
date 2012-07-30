/*
 * Graph.h
 *
 *  Created on: Jan 22, 2010
 *      Author: papazov
 */

#ifndef _TUM_UNDIRECTED_GRAPH_H_
#define _TUM_UNDIRECTED_GRAPH_H_

#include "GraphNode.h"
#include "GraphNodeData.h"
#include <cstdio>
#include <string>
#include <list>
#include <map>

using namespace std;

namespace tum
{

/** This is an implementation of an undirected graph. */
template <class _Key_, class _Compare_>
class UndirectedGraph
{
public:
	UndirectedGraph();
	virtual ~UndirectedGraph();

	/** Adds 'node_pair' and its 'neighbours' to this graph. The method checks whether the nodes specified by the keys
	 * given in 'node_pair' and 'neighbours' already exist in the graph or not and creates new nodes only for the new keys.
	 * At the end the node given by 'node_pair' will be neighbour with the nodes in 'neighbours'. If 'neighbours' is NULL
	 * the method will just insert the node given by 'node_pair' in the graph (if the key of 'node_pair' is not already
	 * in use). If 'deleteNotInsertedData' is 'true' then the method will delete each 'GraphNodeData' object (provided by
	 * the user in 'node' and 'neighbours') which is not inserted in to the graph because the corresponding key is already
	 * in use. */
	void addNodes(pair<_Key_, GraphNodeData*> node, list<pair<_Key_, GraphNodeData*> > *neighbours, bool deleteNotInsertedData);

	int getNumberOfNodes(){ return mAllNodes.size();}
	map<_Key_, GraphNode<_Key_,_Compare_>*, _Compare_>& getNodes(){ return mAllNodes;}

	/** The method constructs a string which can be visualized with the graphviz library. */
	inline void getGraphVizString(string& graphVizStr);

	/** The method constructs a string in the DOT format and saves it in 'graphVizStr' which can be visualized with the
	 * graphviz library. Each edge is contained only once in the string, i.e., if A is connected with B 'graphViz' will
	 * contain either A--B or B--A, but NOT both! */
	inline void getGraphVizStringSingleEdges(string& graphVizStr);

	inline void print(FILE* stream, const char* label = NULL);
	inline void printShort(FILE* stream, const char* label = NULL);

	/** Deletes all nodes. */
	inline void clear();

protected:
	/** This list contains one node per independent component of this graph. The nodes can be used
	 * as entry points for the independent components of the graph. */
//	list<GraphNode*> mComponents;

	/** This is a map of all nodes in this graph. It is useful when some operation needs to be done
	 * with all nodes of the graph. */
	map<_Key_, GraphNode<_Key_,_Compare_>*, _Compare_> mAllNodes;

	int mGraphNodeLabel;
};

}//namespace tum

//=== inline methods ===========================================================================================================

template<class _Key_, class _Compare_>
tum::UndirectedGraph<_Key_, _Compare_>::UndirectedGraph()
{
	mGraphNodeLabel = 1;
}

//==============================================================================================================================

template<class _Key_, class _Compare_>
tum::UndirectedGraph<_Key_, _Compare_>::~UndirectedGraph()
{
	this->clear();
}

//==============================================================================================================================

template<class _Key_, class _Compare_>
inline void tum::UndirectedGraph<_Key_, _Compare_>::clear()
{
	typename map<_Key_, GraphNode<_Key_,_Compare_>*, _Compare_>::iterator it;

	for ( it = mAllNodes.begin() ; it != mAllNodes.end() ; ++it )
		delete it->second;
	mAllNodes.clear();
	mGraphNodeLabel = 1;
}

//==============================================================================================================================

template<class _Key_, class _Compare_>
inline void tum::UndirectedGraph<_Key_, _Compare_>::printShort(FILE* stream, const char* label)
{
	typename map<_Key_, GraphNode<_Key_,_Compare_>*, _Compare_>::iterator node;

	if ( label ) fprintf(stream, "%s\n{\n", label);
	else fprintf(stream, "undirected graph\n{\n");

	for ( node = mAllNodes.begin() ; node != mAllNodes.end() ; ++node )
	{
		if ( !node->second->getData() )
		{
			fprintf(stream, "ERROR: there is a node without data!\n");
			return;
		}
		fprintf(stream, "  '%s' has %i neighbour(s)\n", node->second->getData()->getLabel(), node->second->getNumberOfNeighbours());
	}
	fprintf(stream, "}\n");
	fflush(stream);
}

//==============================================================================================================================

template<class _Key_, class _Compare_>
inline void tum::UndirectedGraph<_Key_, _Compare_>::print(FILE* stream, const char* label)
{
	typename map<_Key_, GraphNode<_Key_,_Compare_>*, _Compare_>::iterator node;

	if ( label ) fprintf(stream, "%s\n{\n", label);
	else fprintf(stream, "undirected graph\n{\n");

	for ( node = mAllNodes.begin() ; node != mAllNodes.end() ; ++node )
	{
		map<_Key_,GraphNode<_Key_,_Compare_>*,_Compare_>& neighbours = node->second->getNeighbours();
		typename map<_Key_,GraphNode<_Key_,_Compare_>*,_Compare_>::iterator neigh;

		if ( !node->second->getData() )
		{
			fprintf(stream, "ERROR: there is a node without data!\n");
			return;
		}
		fprintf(stream, "  %s: ", node->second->getData()->getLabel());

		for ( neigh = neighbours.begin() ; neigh != neighbours.end() ; ++neigh )
		{
			if ( !neigh->second->getData() )
			{
				fprintf(stream, "ERROR: there is a node without data!\n");
				return;
			}
			fprintf(stream, " %s ", neigh->second->getData()->getLabel());
		}
		fprintf(stream, "\n");
	}
	fprintf(stream, "}\n");
	fflush(stream);
}

//==============================================================================================================================

template<class _Key_, class _Compare_>
void tum::UndirectedGraph<_Key_, _Compare_>::addNodes(pair<_Key_, GraphNodeData*> node_pair, list<pair<_Key_, GraphNodeData*> > *neighbours,
		bool deleteNotInsertedData)
{
	// Some variables
	GraphNode<_Key_,_Compare_>* node = new GraphNode<_Key_,_Compare_>(node_pair.first);
	pair<typename map<_Key_, GraphNode<_Key_,_Compare_>*>::iterator, bool> insertResult;

	// Try to insert the new node
	insertResult = mAllNodes.insert(pair<_Key_,GraphNode<_Key_,_Compare_>*>(node->getKey(), node));

	// If 'true' then 'node' was inserted otherwise (i.e., 'false') its key already exists
	if ( insertResult.second )
		node->setData(node_pair.second);
	else
	{
		// There is already a graph node with this key -> delete 'node'
		delete node;
		if ( deleteNotInsertedData ) delete node_pair.second;
		// Get the existing graph node
		node = insertResult.first->second;
	}

	// Check if we should add some neighbours
	if ( neighbours )
	{
		for ( typename list<pair<_Key_,GraphNodeData*> >::iterator it = neighbours->begin() ; it != neighbours->end() ; ++it )
		{
			// Create a new neighbour candidate
			GraphNode<_Key_,_Compare_>* neigh_node = new GraphNode<_Key_,_Compare_>(it->first);
			// Try to add 'neigh_node' to the graph
			insertResult = mAllNodes.insert(pair<_Key_,GraphNode<_Key_,_Compare_>*>(neigh_node->getKey(), neigh_node));
			if ( insertResult.second )
				// The node was really inserted to the graph
				neigh_node->setData(it->second);
			else
			{
				// There already exists a node with key 'it->first'
				delete neigh_node;
				if ( deleteNotInsertedData ) delete it->second;
				// Get the existing node
				neigh_node = insertResult.first->second;
			}

			// Add 'neigh_node' as neighbour of 'node'
			node->addNeighbour(neigh_node);
			// Add 'node' as neighbour of 'neigh_node'
			neigh_node->addNeighbour(node);
		}
	}
}

//==============================================================================================================================

template<class _Key_, class _Compare_>
inline void tum::UndirectedGraph<_Key_, _Compare_>::getGraphVizString(string& graphVizStr)
{
	graphVizStr += "graph undirected \n{\n";
	GraphNodeData* data;
	char node_str[GND_MAX_LABEL_SIZE], neigh_str[GND_MAX_LABEL_SIZE];

	for ( typename map<_Key_,GraphNode<_Key_,_Compare_>*>::iterator it = mAllNodes.begin() ; it != mAllNodes.end() ; ++it )
	{
		data = it->second->getData();
		if ( data )
		{
			if ( data->getLabel()[0] == '\0' )
				sprintf(data->getLabel(), "%i", mGraphNodeLabel++);
			sprintf(node_str, "%s", data->getLabel());
		}
		else
			sprintf(node_str, "%i", mGraphNodeLabel++);

		map<_Key_,GraphNode<_Key_,_Compare_>*>& neighs = it->second->getNeighbours();

		if ( neighs.size() )
		{
			typename map<_Key_,GraphNode<_Key_,_Compare_>*>::iterator neigh_it;
			graphVizStr += '\t';

			for ( neigh_it = neighs.begin() ; neigh_it != neighs.end() ; ++neigh_it )
			{
				data = neigh_it->second->getData();
				if ( data )
				{
					if ( data->getLabel()[0] == '\0' )
						sprintf(data->getLabel(), "%i", mGraphNodeLabel++);
					sprintf(neigh_str, "%s", data->getLabel());
				}
				else
					sprintf(neigh_str, "%i", mGraphNodeLabel++);

				graphVizStr += node_str;
				graphVizStr += "--";
				graphVizStr += neigh_str;
				graphVizStr += "; ";
			}
			graphVizStr += '\n';
		}
		else
		{
			graphVizStr += '\t';
			graphVizStr += node_str;
			graphVizStr += ";\n";
		}
	}
	graphVizStr += "}\n";
}

//==============================================================================================================================

template<class _Key_, class _Compare_>
inline void tum::UndirectedGraph<_Key_, _Compare_>::getGraphVizStringSingleEdges(string& graphVizStr)
{
	typename map<_Key_,GraphNode<_Key_,_Compare_>*>::iterator it;
	GraphNodeData* data;
	char node_str[GND_MAX_LABEL_SIZE], neigh_str[GND_MAX_LABEL_SIZE];
	int i;

	graphVizStr += "graph undirected \n{\n";

	// Just set the node ids
	for ( i = 0, it = mAllNodes.begin() ; it != mAllNodes.end() ; ++it, ++i )
		it->second->setTmpId(i);

	for ( i = 0, it = mAllNodes.begin() ; it != mAllNodes.end() ; ++it, ++i )
	{
		data = it->second->getData();
		if ( data )
		{
			if ( data->getLabel()[0] == '\0' )
				sprintf(data->getLabel(), "%i", it->second->getTmpId());
			sprintf(node_str, "%s", data->getLabel());
		}
		else
			sprintf(node_str, "%i", it->second->getTmpId());

		map<_Key_,GraphNode<_Key_,_Compare_>*>& neighs = it->second->getNeighbours();

		if ( neighs.size() )
		{
			typename map<_Key_,GraphNode<_Key_,_Compare_>*>::iterator neigh_it;
			graphVizStr += '\t';

			for ( neigh_it = neighs.begin() ; neigh_it != neighs.end() ; ++neigh_it )
			{
				if ( neigh_it->second->getTmpId() < i )
					continue;

				data = neigh_it->second->getData();
				if ( data )
				{
					if ( data->getLabel()[0] == '\0' )
						sprintf(data->getLabel(), "%i", neigh_it->second->getTmpId());
					sprintf(neigh_str, "%s", data->getLabel());
				}
				else
					sprintf(neigh_str, "%i", neigh_it->second->getTmpId());

				graphVizStr += node_str;
				graphVizStr += "--";
				graphVizStr += neigh_str;
				graphVizStr += "; ";
			}
			graphVizStr += '\n';
		}
		else
		{
			graphVizStr += '\t';
			graphVizStr += node_str;
			graphVizStr += ";\n";
		}
	}
	graphVizStr += "}\n";
}

//==============================================================================================================================

#endif /* _TUM_UNDIRECTED_GRAPH_H_ */
