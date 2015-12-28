/*
 * Stopwatch.cpp
 *
 *  Created on: Apr 13, 2010
 *      Author: papazov
 */

#include "Stopwatch.h"

#ifdef __APPLE__
#include "realtime.hpp"
#endif // __APPLE__

Stopwatch::Stopwatch(bool print_err_msg)
{
#ifdef __APPLE__
#ifdef HAVE_REALTIME
        int msToNS = 1000000, period = 1*msToNS, computation = 0.1*msToNS, constraint = 0.5*msToNS;
         bool preemptible = false;
        if(set_realtime( period,  computation,  constraint,  preemptible)){
           std::cout << "realtime mode successfully enabled!\n";
        };
#endif // HAVE_REALTIME
#else // __APPLE__
    // LINUX
	// Measure test time
	sched_param mySchedParam;
	mySchedParam.__sched_priority =	sched_get_priority_max(SCHED_FIFO);

	if ( mySchedParam.__sched_priority == -1 )
	{
		if ( print_err_msg )
			printf("Stopwatch::Stopwatch(): sched_get_priority_max() failed\n");
		mySchedParam.__sched_priority = 99;
	}
	if ( sched_setscheduler(0, SCHED_FIFO, &mySchedParam) == -1 && print_err_msg )
	{
		printf("Stopwatch::Stopwatch(): Changing to real-time priority level %i failed. ", mySchedParam.__sched_priority);
		switch ( errno )
		{
			case EINVAL:
				printf("Reason: Unknown policy.\n");
				break;
			case EPERM:
				printf("Reason: Process does not have appropriate privileges. Run as root.\n");
				break;
			case ESRCH:
				printf("Reason: Unknown process.\n");
				break;
			case ENOSYS:
				printf("Reason: Function not supported.\n");
			default:
				printf("Unknown error code: %i.\n", errno);
				break;
		}
	}
#endif
	fflush(stdout);
}

Stopwatch::~Stopwatch()
{
}
