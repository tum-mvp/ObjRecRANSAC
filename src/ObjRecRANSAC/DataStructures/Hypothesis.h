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
  Hypothesis() :
    pair_id(-1),
    model_entry(NULL)
  { }
  virtual ~Hypothesis(){
  }

public:
  double rigid_transform[12];
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
