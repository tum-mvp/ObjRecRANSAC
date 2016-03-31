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

#ifdef __APPLE__
#include <mach/mach_time.h>
#define ORWL_NANO (+1.0E-9)
#define ORWL_GIGA UINT64_C(1000000000)

static double orwl_timebase = 0.0;
static uint64_t orwl_timestart = 0;

inline struct timespec orwl_gettime(void) {
  // be more careful in a multithreaded environement
  if (!orwl_timestart) {
    mach_timebase_info_data_t tb = { 0 };
    mach_timebase_info(&tb);
    orwl_timebase = tb.numer;
    orwl_timebase /= tb.denom;
    orwl_timestart = mach_absolute_time();
  }
  struct timespec t;
  double diff = (mach_absolute_time() - orwl_timestart) * orwl_timebase;
  t.tv_sec = diff * ORWL_NANO;
  t.tv_nsec = diff - (t.tv_sec * ORWL_GIGA);
  return t;
}
#endif // __APPLE__

class Stopwatch
{
public:
	Stopwatch(bool print_err_msg = true);
	virtual ~Stopwatch();

	void start(){
#ifdef __APPLE__
     mStart = orwl_gettime();
#else
	 clock_gettime(CLOCK_REALTIME, &mStart);
#endif
    }

	/** Returns the elapsed time in seconds since the last call of 'this->start()'. */
	inline double stop();

protected:
	timespec mStart;
};

//=== inline methods =============================================================================================================

inline double Stopwatch::stop()
{
    timespec stop;
#ifdef __APPLE__
     stop = orwl_gettime();
#else
	 clock_gettime(CLOCK_REALTIME, &stop);
#endif
	// Return the difference in seconds
	return (double)(stop.tv_sec - mStart.tv_sec) + ((double)(stop.tv_nsec - mStart.tv_nsec))/1000000000.0;
}

#endif /* STOPWATCH_H_ */
