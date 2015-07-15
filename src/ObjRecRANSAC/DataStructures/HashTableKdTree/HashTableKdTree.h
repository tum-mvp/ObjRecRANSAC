/*
 * HashTableORRKdTree.h
 *
 *  Created on: Jan 12, 2010
 *      Author: papazov
 */

#ifndef HASHTABLE_KDTREE_H_
#define HASHTABLE_KDTREE_H_

#include "../ORRKdTree/ORRKdTree.h"
#include "../DatabaseModelEntry.h"

class HashTableKdTree: public ORRKdTree
{
public:
	/** Creates the hash table and adds 'dbModelEntry' to it by inserting the 'points' to the appropriate cells
	  * of this kd-tree. The cells will have sizes defined by 'cellSize', which has to be an array of length 'dimension'.
	  * 'points' has to be a 'numOfPoints' x 'dimension' matrix. 'boundingBox' is the bounding box of the 'points'
	  * and has to be a 'dimension' x 2 matrix. If 'boundingBox' is NULL it is computed inside the method. */
	HashTableKdTree(DatabaseModelEntry* dbModelEntry, double** points, int numOfPoints,
			double* cellSize, double dimension, double** boundingBox);
	virtual ~HashTableKdTree();

	/** Adds 'dbModelEntry' to this hash table by inserting the 'points' to the appropriate cells of this kd-tree.
	  * 'points' has to be a 'numOfPoints' x D matrix, where D is the dimension of this kd-tree.
	  * 'boundingBox' is the bounding box of the 'points' and has to be a D x 2 matrix. If 'boundingBox' is NULL
	  * it is computed inside the method. */
	void addModel(DatabaseModelEntry* dbModelEntry, double** points, int numOfPoints, double** boundingBox);

protected:
	void addPoints(DatabaseModelEntry* dbModelEntry, double** points, int numOfPoints);
};

#endif /* HASHTABLEORRKDTREE_H_ */
