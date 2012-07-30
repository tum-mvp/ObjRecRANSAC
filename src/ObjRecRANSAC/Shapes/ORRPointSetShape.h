#ifndef _GEOMETRIC_SHAPE_H_
#define _GEOMETRIC_SHAPE_H_

#include "PointSetShape.h"
#include "../DataStructures/RangeImage/ORRRangeImage2.h"
#include <BasicToolsL1/Vector.h>
#include <BasicToolsL1/Matrix.h>
#include <vtkPolyData.h>
#include <algorithm>
#include <vector>
#include <list>

using namespace std;

class ORRPointSetShape: public PointSetShape
{
public:
	inline ORRPointSetShape(UserData* userdata, vtkPolyData* polydata, const double* rigid_transform,
			vtkPolyData* highResModel, int instanceNumber);
	virtual ~ORRPointSetShape(){}

	enum SceneState {SCENE_FREE = 0, SCENE_ON, SCENE_OFF};

	int getInstanceNumber(){ return mInstanceNumber;}
	const char* getLabel(){ return mLabel;}
	SceneState getSceneState(){ return mSceneState;}
	void setSceneState(SceneState state){ mSceneState = state;}
	void setSceneStateOff(){ mSceneState = SCENE_OFF;}
	void setSceneStateOn(){ mSceneState = SCENE_ON;}
	bool isSceneStateOff(){ return mSceneState == SCENE_OFF;}
	bool isSceneStateOn(){ return mSceneState == SCENE_ON;}

	inline void getPoint1(double* p)const{ vec_copy3(m_p1, p);}
	inline void getPoint2(double* p)const{ vec_copy3(m_p2, p);}
	inline void getPoint3(double* p)const{ vec_copy3(m_p3, p);}
	inline void getNormal1(double* n)const{ vec_copy3(m_n1, n);}
	inline void getNormal2(double* n)const{ vec_copy3(m_n2, n);}
	inline void getNormal3(double* n)const{ vec_copy3(m_n3, n);}

	const double* getPoint1(){ return m_p1;}
	const double* getPoint2(){ return m_p2;}
	const double* getPoint3(){ return m_p3;}
	const double* getNormal1(){ return m_n1;}
	const double* getNormal2(){ return m_n2;}
	const double* getNormal3(){ return m_n3;}

	inline void setPoint1(const double* p){ vec_copy3(p, m_p1);}
	inline void setPoint2(const double* p){ vec_copy3(p, m_p2);}
	inline void setPoint3(const double* p){ vec_copy3(p, m_p3);}
	inline void setNormal1(const double* n){ vec_copy3(n, m_n1);}
	inline void setNormal2(const double* n){ vec_copy3(n, m_n2);}
	inline void setNormal3(const double* n){ vec_copy3(n, m_n3);}

	inline void addPixel(const ORRRangeImage2* image, int x, int y);
	int getNumberOfOccupiedScenePixels(){ return (int)mLinearPixelIds.size();}
	list<OctreeNode*>& getOctreeSceneNodes(){ return mSceneOctreeNodes;}

	inline void sortLinearPixelIds();
	const vector<int>& getLinearPixelIds(){ return mLinearPixelIds;}

	void setShapeId(int id){ mShapeId = id;}
	int getShapeId()const{ return mShapeId;}

protected:
	int mInstanceNumber;
	char mLabel[2048];
	SceneState mSceneState;
	double m_p1[3], m_p2[3], m_p3[3], m_n1[3], m_n2[3], m_n3[3];
	int mShapeId;
	// Needed in order to get the point ids of the points contained in each node
	list<OctreeNode*> mSceneOctreeNodes;
	vector<int> mLinearPixelIds;
};

//=== inline methods ==================================================================================================================

inline ORRPointSetShape::ORRPointSetShape(UserData* userdata, vtkPolyData* polydata, const double* rigid_transform,
		vtkPolyData* highResModel, int instanceNumber)
: PointSetShape(userdata, polydata, rigid_transform, highResModel)
{
	mInstanceNumber = instanceNumber;
	mSceneState = ORRPointSetShape::SCENE_FREE;
	mShapeId = -1;

	if ( mUserData )
		sprintf(mLabel, "%s_%i", mUserData->getLabel(), mInstanceNumber);
	else
		sprintf(mLabel, "point set shape %i", mInstanceNumber);
}

//=====================================================================================================================================

inline void ORRPointSetShape::addPixel(const ORRRangeImage2* image, int x, int y)
{
	// Get the scene nodes at the pixel [x, y]
	const list<OctreeNode*>* nodes = image->getOctreeNodes()[x][y];
	if ( !nodes )
	{
		fprintf(stderr, "WARNING in 'ORRPointSetShape::%s()': there should be a list of nodes at pixel [%i, %i].\n", __func__, x, y);
		fflush(stderr);
		return;
	}

	// Save the linear id
	mLinearPixelIds.push_back(image->getLinearId(x, y));

	// Save the nodes
	for ( list<OctreeNode*>::const_iterator node = nodes->begin() ; node != nodes->end() ; ++node )
		mSceneOctreeNodes.push_back(*node);
}

//=====================================================================================================================================

inline bool compareIntFunc(int i,int j) { return (i<j); }

//=====================================================================================================================================

inline void ORRPointSetShape::sortLinearPixelIds()
{
	sort(mLinearPixelIds.begin(), mLinearPixelIds.end(), compareIntFunc);
}

//=====================================================================================================================================

#endif /*_GEOMETRIC_SHAPE_H_*/
