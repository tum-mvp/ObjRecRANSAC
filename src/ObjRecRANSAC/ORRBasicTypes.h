/*
 * ORRBasicTypes.h
 *
 *  Created on: Jun 9, 2010
 *      Author: papazov
 */

#ifndef _ORR_BASIC_TYPES_H_
#define _ORR_BASIC_TYPES_H_

class int_2
{
public:
	int_2(){x = y = -1;}
	int x, y;
};

typedef struct _double_2_
{
	double x, y;
}double_2;

class double_3
{
public:
	double_3(const double* u){x = u[0]; y = u[1]; z = u[2];}
	double x, y, z;
};

#endif /* _ORR_BASIC_TYPES_H_ */
