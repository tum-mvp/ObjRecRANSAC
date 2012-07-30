/*
 * Stopwatch.h
 *
 *  Created on: Apr 13, 2010
 *      Author: papazov
 */

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include <stdio.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>
#include <sched.h>

class Stopwatch
{
public:
	Stopwatch(bool print_err_msg = true);
	virtual ~Stopwatch();

	void start(){ clock_gettime(CLOCK_REALTIME, &mStart);}

	/** Returns the elapsed time in seconds since the last call of 'this->start()'. */
	inline double stop();

protected:
	timespec mStart;
};

//=== inline methods =============================================================================================================

inline double Stopwatch::stop()
{
	timespec stop; clock_gettime(CLOCK_REALTIME, &stop);
	// Return the difference in seconds
	return (double)(stop.tv_sec - mStart.tv_sec) + ((double)(stop.tv_nsec - mStart.tv_nsec))/1000000000.0;
}

#endif /* STOPWATCH_H_ */
