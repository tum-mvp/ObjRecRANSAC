#include "Box.h"

Box::Box()
{
	mIntervals = NULL;
	mDimension = 0;
}

Box::Box(int dimension)
{
	mIntervals = NULL;
	mDimension = 0;
	this->alloc(dimension);
}

Box::Box(const Box& box)
{
	mIntervals = NULL;
	mDimension = 0;
	this->alloc(box.mDimension);
	this->copyfrom(box);
}

Box::~Box()
{
	this->dealloc();
}

//============================================================================================================================

void Box::print(FILE* dst)
{
	for ( int i = 0 ; i < mDimension ; ++i )
		fprintf(dst, "[%16.6lf, %16.6lf]\n", mIntervals[i][0], mIntervals[i][1]);
}

//============================================================================================================================
