/*
 * HashTableKdTree.cpp
 *
 *  Created on: Jan 12, 2010
 *      Author: papazov
 */

#include "HashTableKdTree.h"
#include <BasicTools/ComputationalGeometry/Algorithms/AuxCompGeom.h>
#include "HashTableCellKd.h"
#include "CellEntry.h"
#include <cstdio>


HashTableKdTree::HashTableKdTree(DatabaseModelEntry* dbModelEntry, double** points, int numOfPoints,
		double* cellSize, double dimension, double** boundingBox)
{
	Box bbox(dimension);
	// Copy the bounding box or compute it by your own
	if ( boundingBox ) bbox.copyfrom((const double**)boundingBox);
	else AuxCompGeom::boundingBox(numOfPoints, points, dimension, bbox.mIntervals);

	// Enlarge the bounding box a bit in order to avoid points lying exactly on the boundary of the kd-tree
	AuxCompGeom::enlarge(bbox.mIntervals, dimension, 0.000001);
	// Build the kd-tree
	this->buildORRKdTree(dimension, cellSize, bbox.mIntervals);

	// Add the points to the tree
	this->addPoints(dbModelEntry, points, numOfPoints);
}

HashTableKdTree::~HashTableKdTree()
{
}

//===================================================================================================================================

void HashTableKdTree::addModel(DatabaseModelEntry* dbModelEntry, double** points, int numOfPoints, double** boundingBox)
{
	int dimension = this->getDimension();

	Box bbox(dimension);
	// Copy the bounding box or compute it by your own
	if ( boundingBox ) bbox.copyfrom((const double**)boundingBox);
	else AuxCompGeom::boundingBox(numOfPoints, points, dimension, bbox.mIntervals);

	// Enlarge the bounding box a bit in order to avoid points lying exactly on the boundary of the kd-tree
	AuxCompGeom::enlarge(bbox.mIntervals, dimension, 0.000001);
	// Extend the tree if necessary such that it covers the bounding box of 'points'
	this->extendTree(bbox.mIntervals);
	// Add the points to the tree
	this->addPoints(dbModelEntry, points, numOfPoints);
}

//===================================================================================================================================

void HashTableKdTree::addPoints(DatabaseModelEntry* dbModelEntry, double** points, int numOfPoints)
{
	bool newLeafWasCreated;
	KdTreeNode* leaf;
	HashTableCellKd* cell;
	CellEntry* cellEntry;

	for ( int i = 0 ; i < numOfPoints ; ++i )
	{
		// Get the leaf which contains 'points[i]'
		leaf = this->createLeaf(points[i], newLeafWasCreated);
		if ( !leaf->getData() )
			leaf->setData(new HashTableCellKd());

		// Get the cell
		cell = (HashTableCellKd*)leaf->getData();
		// Create a new entry and save it in the cell
		cellEntry = new CellEntry(dbModelEntry, points[i]);
		cell->addEntry(cellEntry);
	}
}

//===================================================================================================================================
