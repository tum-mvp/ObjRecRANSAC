#ifndef NDBOXSTRUCTURE_H_
#define NDBOXSTRUCTURE_H_

#include <cstdio>

template <class T>
class NdBoxStructure
{
public:
	inline NdBoxStructure();
	inline virtual ~NdBoxStructure();

	inline bool buildNdBoxStructure(int* numOfBoxes, int dimension, double** structBounds);
	inline virtual void clear();

	inline int getDimension()const{ return mDimension;}
	inline void getBoxLengths(double *lengths) const;
	inline void getNumberOfBoxes(int *numOfBoxes) const;
	inline void getStructureBounds(double** bounds) const;
	/** Saves pointers to the full neighbors of the cell which contains 'point'. 'out' has
	  * to be an array of at least 3^mDimension pointers of type 'T'. This is the maximal
	  * number of full neighbors a box can have. The method returns the number of full neighbors. */
	inline int getNeighbors(const double* point, T** out) const;
	T** getData()const{ return mData;}

	inline void setBox(const int* id, T* data);

	inline int getBoxId(const double* point) const;
	inline int getBoxId(const int* id) const;
	inline T* getBox(int id)const{ return mData[id];}
	inline T* getBox(const double* point) const;
	/** Make sure that the n-dimensional 'id' is valid! Otherwise the method will crash! */
	inline T* getBox(const int* id) const;
	inline bool nextId(int* id, const int* beg, const int* end) const;

protected:
	T **mData;
	double *mBoxLengths, **mStructBounds;
	int mDimension, mNumOfDataItems, *mNumOfBoxes, *mSpecialNumOfBoxes;
};

//===========================================================================================================================

template<class T>
NdBoxStructure<T>::NdBoxStructure()
{
	mData = NULL;
	mStructBounds = NULL;
	mBoxLengths = NULL;
	mNumOfBoxes = NULL;
	mSpecialNumOfBoxes = NULL;
	mDimension = -1;
	mNumOfDataItems = 0;
}

template<class T>
NdBoxStructure<T>::~NdBoxStructure()
{
	this->clear();
}

//===========================================================================================================================

template<class T>
void NdBoxStructure<T>::clear()
{
	int i;

	if ( mData )
	{
		for ( i = 0 ; i < mNumOfDataItems ; ++i )
			if ( mData[i] ) delete mData[i];
	
		delete[] mData;
		mData = NULL;
	}

	if ( mStructBounds )
	{
		for ( i = 0 ; i < mDimension ; ++i )
			delete[] mStructBounds[i];
		delete[] mStructBounds;
		mStructBounds = NULL;
	}

	if ( mBoxLengths ){ delete[] mBoxLengths; mBoxLengths = NULL;}
	if ( mNumOfBoxes ){ delete[] mNumOfBoxes; mNumOfBoxes = NULL;}
	if ( mSpecialNumOfBoxes ){ delete[] mSpecialNumOfBoxes; mSpecialNumOfBoxes = NULL;}

	mDimension = -1;
	mNumOfDataItems = 0;
}

//===========================================================================================================================

template <class T>
inline void NdBoxStructure<T>::getBoxLengths(double *lengths) const
{
	for ( int i = 0 ; i < mDimension ; ++i )
		lengths[i] = mBoxLengths[i];
}

//===========================================================================================================================

template <class T>
inline void NdBoxStructure<T>::getNumberOfBoxes(int *numOfBoxes) const
{
	for ( int i = 0 ; i < mDimension ; ++i )
		numOfBoxes[i] = mNumOfBoxes[i];
}

//===========================================================================================================================

template <class T>
inline void NdBoxStructure<T>::getStructureBounds(double** bounds) const
{
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		bounds[i][0] = mStructBounds[i][0];
		bounds[i][1] = mStructBounds[i][1];
	}
}

//===========================================================================================================================

template<class T>
bool NdBoxStructure<T>::buildNdBoxStructure(int* numOfBoxes, int dimension, double** structBounds)
{
	this->clear();

	// Alloc memory
	mStructBounds = new double*[dimension];
	mBoxLengths = new double[dimension];
	mNumOfBoxes = new int[dimension];
	mSpecialNumOfBoxes = new int[dimension];

	// Init
	mDimension = dimension;
	mNumOfDataItems = 1;

	int i;
	for ( i = 0 ; i < dimension  ; ++i )
	{
		// Fix the global structure bounds
		mStructBounds[i] = new double[2];
		mStructBounds[i][0] = structBounds[i][0];
		mStructBounds[i][1] = structBounds[i][1];
		// Get the number of boxes along the current axis
		mNumOfBoxes[i] = numOfBoxes[i];
		// Fix the box length
		mBoxLengths[i] = (structBounds[i][1] - structBounds[i][0])/(double)numOfBoxes[i];
		// Compute the number of data items
		mNumOfDataItems *= numOfBoxes[i];
	}

	mSpecialNumOfBoxes[0] = 1;
	for ( i = 0 ; i < dimension-1 ; ++i )
		mSpecialNumOfBoxes[i+1] = mSpecialNumOfBoxes[i]*mNumOfBoxes[i];

	// Alloc memory for the pointer array
	try
	{
		mData = new T*[mNumOfDataItems];
		// Set all pointers to NULL
		for ( i = 0 ; i < mNumOfDataItems ; ++i )
			mData[i] = NULL;
	}
	catch ( ... )
	{
		fprintf(stderr, "ERROR in 'NdBoxStructure::%s()': can not allocate %i bytes.\n",
				__func__, (int)(sizeof(T*)*mNumOfDataItems)); fflush(stdout);
		mData = NULL;
		return false;
	}

	return true;
}

//====================================================================================================================================

template<class T>
inline int NdBoxStructure<T>::getBoxId(const int* id) const
{
	int i, linId;

	for ( i = 0, linId = 0 ; i < mDimension ; ++i )
		linId += id[i]*mSpecialNumOfBoxes[i];

	return linId;
}

//====================================================================================================================================

template<class T>
inline int NdBoxStructure<T>::getBoxId(const double* point) const
{
	int i, id;

	for ( i = 0, id = 0 ; i < mDimension ; ++i )
	{
		if ( point[i] < mStructBounds[i][0] || point[i] >= mStructBounds[i][1] )
			return -1;

		id += mSpecialNumOfBoxes[i]*(int)((point[i]-mStructBounds[i][0])/mBoxLengths[i]);
	}

	return id;
}

//====================================================================================================================================

template<class T>
inline T* NdBoxStructure<T>::getBox(const double* point) const
{
	int id = this->getBoxId(point);
	if ( id < 0 )
		return NULL;

	return mData[id];
}

//====================================================================================================================================

template<class T>
inline T* NdBoxStructure<T>::getBox(const int* id) const
{
	int i, tmp;
	for ( i = 0, tmp = 0 ; i < mDimension ; ++i )
		tmp += mSpecialNumOfBoxes[i]*id[i];

	return mData[tmp];
}

//====================================================================================================================================

template<class T>
inline int NdBoxStructure<T>::getNeighbors(const double* point, T** out) const
{
	int i, tmpId, *tmp_id = new int[mDimension], *tmp_id_beg = new int[mDimension], *tmp_id_end = new int[mDimension];

	// Compute the n-dimensional id and check whether the point is within the box structure
	for ( i = 0 ; i < mDimension ; ++i )
	{
		tmpId = (int)((point[i]-mStructBounds[i][0])/mBoxLengths[i]);
		if ( tmpId < 0 || tmpId >= mNumOfBoxes[i] )
			return 0;

		tmp_id_beg[i] = tmpId - 1;
		tmp_id_end[i] = tmpId + 1;

		if ( tmp_id_beg[i] < 0 )
			tmp_id_beg[i] = 0;
		if ( tmp_id_end[i] == mNumOfBoxes[i] )
			--tmp_id_end[i];

		// Init
		tmp_id[i] = tmp_id_beg[i];
	}

	T* box;	i = 0;
	// Generate all valid ids
	do
	{
		box = this->getBox(tmp_id);
		if ( box ) out[i++] = box;
	}
	while ( this->nextId(tmp_id, tmp_id_beg, tmp_id_end) );

	// Cleanup
	delete[] tmp_id;
	delete[] tmp_id_beg;
	delete[] tmp_id_end;

	// Return the number of full boxes
	return i;
}

//====================================================================================================================================

template<class T>
inline bool NdBoxStructure<T>::nextId(int* id, const int* beg, const int* end) const
{
	for ( int i = 0 ; i < mDimension ; ++i )
	{
		if ( id[i] < end[i] )
		{
			++id[i];
			return true;
		}
		else
			id[i] = beg[i];
	}
	return false;
}

//====================================================================================================================================

template<class T>
inline void NdBoxStructure<T>::setBox(const int* id, T* data)
{
	int linId = this->getBoxId(id);
	if ( mData[linId] )
		delete mData[linId];
	mData[linId] = data;
}

//====================================================================================================================================

#endif /*NDBOXSTRUCTURE_H_*/
