/*
 * Hypothesis.h
 *
 *  Created on: Apr 9, 2011
 *      Author: papazov
 */

#ifndef HYPOTHESIS_H_
#define HYPOTHESIS_H_

class DatabaseModelEntry;

class Hypothesis
{
public:
	Hypothesis(double* rigid_transform, int pair_id, DatabaseModelEntry* model_entry)
	{
		this->rigid_transform = rigid_transform;
		this->pair_id = pair_id;
		this->model_entry = model_entry;
	}
	virtual ~Hypothesis(){ delete[] rigid_transform;}

public:
	double* rigid_transform;
	int pair_id;
	DatabaseModelEntry* model_entry;
};

//===================================================================================================================

class AcceptedHypothesis
{
public:
	AcceptedHypothesis(){ rigid_transform = NULL; match = 0; model_entry = NULL;}
	virtual ~AcceptedHypothesis(){}

public:
	double *rigid_transform;
	int match;
	DatabaseModelEntry* model_entry;
};

//===================================================================================================================

#endif /* HYPOTHESIS_H_ */
