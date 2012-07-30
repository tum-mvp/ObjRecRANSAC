#ifndef _TUM_OCTREENODE_H_
#define _TUM_OCTREENODE_H_

#include "OctreeNodeData.h"

namespace tum
{

class OctreeNode
{
public:
	OctreeNode();
	virtual ~OctreeNode();

	void setParent(OctreeNode* parent){ mParent = parent;}
	OctreeNode* getParent(){ return mParent;}

	bool hasChildren()const{ return (bool)mChildren;}
	void createChildren();
	OctreeNode* getChildren(){ return mChildren;}
	OctreeNode* getChild(int id){ return &mChildren[id];}
	void destroyChildren();

	bool isFull()const{ return (bool)mData;}
	bool hasData()const{ return (bool)mData;}
	/** Note that this method calls the destructor of 'mData' before it assigns 'data' to it. Calling the
	 * destructor of this OctreeNode object will lead to the destruction of 'mData', i.e., of 'data' too.
	 * So, once you have called this method, you do NOT need to call the destructor of 'data' by your own. */
	void setData(OctreeNodeData* data);
	OctreeNodeData* getData(){ return mData;}
	void destroyData();

	unsigned char getFullDescendantsFlags(){ return mFullDescendantsFlags;}
	void setFullDescendantsFlags(unsigned char mask){ mFullDescendantsFlags = mask;}
	void fullDescendantsFlagsBitwiseOR(unsigned char mask){ mFullDescendantsFlags |= mask;}
	void fullDescendantsFlagsBitwiseAND(unsigned char mask){ mFullDescendantsFlags &= mask;}

	inline void allocNeighbours(int maxNumOfNeighbours);
	void addNeighbour(OctreeNode* neighbour){ mNeighbours[mNumOfNeighbours++] = neighbour;}
	void clearNeighbours(){ if ( mNeighbours ) delete[] mNeighbours; mNumOfNeighbours = 0;}
	OctreeNode** getNeighbours(){ return mNeighbours;}
	inline void getNeighbours(OctreeNode** out);
	int getNumberOfNeighbours(){ return mNumOfNeighbours;}

	unsigned char getNeighCheck(){ return mNeighCheck;}
	void setNeighCheckToTrue(){ mNeighCheck = 1;}
	void setNeighCheckToFalse(){ mNeighCheck = 0;}

	double* getBounds(){ return mBounds;}
	inline void getBounds(double* min, double* max);
	const double* getCenter()const{ return mCenter;}
	void getCenter(double* c){c[0] = mCenter[0]; c[1] = mCenter[1]; c[2] = mCenter[2];}
	void setBounds(const double* bounds);
	void setCenter(const double* center);

	double getNodeSize(){ return mBounds[1]-mBounds[0];}
	double getVoxelRadius(){ return mVoxelRadius;}
	int getChildNr(){ return mChildNr;}
	int getFullLeafId(){ return mFullLeafId;}
	void setFullLeafId(int id){ mFullLeafId = id;}
	unsigned char getTmpFlag(){ return mTmpFlag;}
	void setTmpFlag(unsigned char value){ mTmpFlag = value;}

	void computeMiddlePoint(const double* bounds, double* mp);

protected:
	double mBounds[6], mCenter[3], mVoxelRadius;
	OctreeNode *mChildren, *mParent;
	OctreeNodeData *mData;
	unsigned char mFullDescendantsFlags, mChildNr, mNeighCheck, mTmpFlag;
	OctreeNode **mNeighbours;
	int mNumOfNeighbours, mFullLeafId;
};

}//namespace tum

//=== inline methods ===========================================================================================

inline void tum::OctreeNode::getBounds(double* min, double* max)
{
	min[0] = mBounds[0]; max[0] = mBounds[1];
	min[1] = mBounds[2]; max[1] = mBounds[3];
	min[2] = mBounds[4]; max[2] = mBounds[5];
}

//==============================================================================================================

inline void tum::OctreeNode::allocNeighbours(int maxNumOfNeighbours)
{
	if ( mNeighbours ) delete[] mNeighbours;
	mNeighbours = new OctreeNode*[maxNumOfNeighbours];
	mNumOfNeighbours = 0;
}

//==============================================================================================================

inline void tum::OctreeNode::getNeighbours(OctreeNode** out)
{
	for ( int j = 0 ; j < mNumOfNeighbours ; ++j )
		out[j] = mNeighbours[j];
}

//==============================================================================================================

#endif /*_TUM_OCTREENODE_H_*/
