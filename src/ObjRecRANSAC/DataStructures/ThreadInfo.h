/*
 * ThreadInfo.h
 *
 *  Created on: Apr 10, 2011
 *      Author: papazov
 */

#ifndef THREADINFO_H_
#define THREADINFO_H_

struct ThreadResult
{
	ThreadResult(){ match = 0; model_entry = NULL; transform = NULL;}
	virtual ~ThreadResult(){}

	int match;
	DatabaseModelEntry* model_entry;
	double *transform;
};

/** Each thread has to check %num_transforms transforms. %model_points[k] and %pair_ids[k] belong to the
  * k-th transform, namely %transform[12*k]. %pair_results has to be an array with so many elements as there
  * are pairs sampled from the scene. The result in %pair_result[%pair_ids[k]] belongs to the scene pair with
  * id %pair_ids[k]. */
struct ThreadInfo
{
	int num_transforms;
	double *transforms;
	const double **model_points;
	DatabaseModelEntry **model_entries;
	const int *pair_ids;
  std::vector<ThreadResult> pair_result;
};

#endif /* THREADINFO_H_ */
