/*
 * StringUtils.cpp
 *
 *  Created on: Mar 14, 2010
 *      Author: papazov
 */

#include "StringUtils.h"
#include <ctime>
#include <cstdio>

StringUtils::StringUtils()
{
}

StringUtils::~StringUtils()
{
}

//=================================================================================================================

void StringUtils::getDateAndTime(char* out)
{
	// Construct the file name
	time_t theTime;	time(&theTime);
	struct tm *localTime = localtime(&theTime);
	sprintf(out, "%i_%i_%i__%ih_%im",
			localTime->tm_year+1900, localTime->tm_mon+1, localTime->tm_mday, localTime->tm_hour, localTime->tm_min);
}

//=================================================================================================================
