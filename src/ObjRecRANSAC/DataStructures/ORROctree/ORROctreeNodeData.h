#ifndef ORROCTREENODEDATA_H_
#define ORROCTREENODEDATA_H_

#include <BasicTools/ComputationalGeometry/DataStructures/Octree/OctreeNodeData.h>
#include <cstdio>
#include <list>

using namespace std;
using namespace tum;

class ORROctreeNodeData: public OctreeNodeData
{
public:
	ORROctreeNodeData();
	virtual ~ORROctreeNodeData();

	void addToPoint(double* p){ mPoint[0] += p[0]; mPoint[1] += p[1]; mPoint[2] += p[2]; ++mNumOfPoints;}
	const double* getPoint(){ return mPoint;}
	void getPoint(double* p){ p[0] = mPoint[0]; p[1] = mPoint[1]; p[2] = mPoint[2];}
	double* getNormal(){ return mNormal;}
	void setNormal(double x, double y, double z){ mNormal[0] = x; mNormal[1] = y; mNormal[2] = z;}
	void setNormal(const double* n){ mNormal[0] = n[0]; mNormal[1] = n[1]; mNormal[2] = n[2];}
	void allocNormal(){ if ( mNormal ) return; mNormal = new double[3];}
	void clearNormal(){ if ( mNormal ){delete mNormal; mNormal = NULL;} }
	inline void computeMeanPoint();

	int getOwnPointId(){ return mOwnPointId;}
	void setOwnPointId(int id){ mOwnPointId = id;}

	void addPointId(int id){ mPointIds.push_back(id);}
	list<int>& getPointIds(){ return mPointIds;}

	void set3dId(int i, int j, int k){m3dId[0] = i; m3dId[1] = j; m3dId[2] = k;}
	const int* get3dId(){ return m3dId;}

protected:
	int mNumOfPoints, mOwnPointId, m3dId[3];
	double *mNormal, mPoint[3];
	list<int> mPointIds;
};

//=== inline methods ========================================================================================================

inline void ORROctreeNodeData::computeMeanPoint()
{
	mPoint[0] /= (double)mNumOfPoints;
	mPoint[1] /= (double)mNumOfPoints;
	mPoint[2] /= (double)mNumOfPoints;
	mNumOfPoints = 1;
}

//===========================================================================================================================

#endif /*ORROCTREENODEDATA_H_*/
