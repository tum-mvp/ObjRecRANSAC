#ifndef OBJRECRANSAC_H_
#define OBJRECRANSAC_H_

#include "ORRDefines.h"
#include <BasicToolsL1/Vector.h>
#include <BasicTools/DataStructures/LayeredPointSet.h>
#include <BasicTools/ComputationalGeometry/Algorithms/PCA.h>
#include <BasicTools/ComputationalGeometry/Algorithms/OptimalTransformation.h>
#include <BasicTools/LinearAlgebra/Matrix.h>
#include <BasicTools/LinearAlgebra/Vector.h>
#include <BasicTools/Graph/UndirectedGraph.h>
#include "DataStructures/RangeImage/ORRRangeImage2.h"
#include "Algorithms/HashTableKeyGenerator.h"
#include "Algorithms/GeometryProcessor.h"
#include "Shapes/PointSetShape.h"
#include "Shapes/ORRPointSetShape.h"
#include "ModelDatabase.h"
#include "UserData.h"
#include "DataStructures/HashTableBoxStruct/HashTableBoxStruct.h"
#include "DataStructures/ORROctree/ORROctreeNodeData.h"
#include "DataStructures/ORROctree/ORROctree.h"
#include "DataStructures/Hypothesis.h"
#include <vtkPolyData.h>
//#include <ippm.h>
#include <cstdio>
#include <vector>
#include <cmath>
#include <list>

#include <boost/thread/recursive_mutex.hpp>

using namespace std;
using namespace tum;

class ObjRecRANSAC
{
public:

  class OrientedPair
  {
  public:
    OrientedPair(const double *p1, const double *n1, const double *p2, const double *n2){
      vec_copy3(p1, this->p1); vec_copy3(n1, this->n1); vec_copy3(p2, this->p2); vec_copy3(n2, this->n2);
    }
    OrientedPair(){}
    virtual ~OrientedPair(){}
    double p1[3], n1[3], p2[3], n2[3];
  };

public:
  ObjRecRANSAC(double pairwidth, double voxelsize, double relNumOfPairsInHashTable = 0.8);
  virtual ~ObjRecRANSAC();

  bool addModel(vtkPolyData* model, UserData* userData);
  /** Do NOT forget to delete the shapes saved in 'detectedShapes' after using them! */
  int doRecognition(vtkPoints* scene, double successProbability, list<boost::shared_ptr<PointSetShape> >& detectedShapes);

  /** Usually you do not need to call this method. It will be called within 'this->doRecognition()'. */
  bool buildSceneOctree(vtkPoints* scene, double voxelsize);
  static double dot3(const double_3* u, const double* v){ return (u->x * v[0])  +  (u->y * v[1])  +  (u->z * v[2]);}

  void setVisibility(double value){ mVisibility = value;}
  void setRelativeObjectSize(double value){ mRelativeObjSize = value;}
  void setICPRefinement(bool enable, double epsilon){ mICPRefinement = enable; mICPEpsilon = epsilon; }
  void setRelativeNumberOfIllegalPoints(double value){ mRelativeNumOfIllegalPts = value;}
  /** As fraction of the voxel size. */
  void setZDistanceThreshAsVoxelSizeFraction(double value){ mAbsZDistThresh = value*mVoxelSize;}
  void setNormalEstimationRadius(int value){ mNormalEstimationNeighRadius = value;}
  void setIntersectionFraction(double value){ mIntersectionFraction = value;}
  void setNumberOfThreads(int numOfThreads){ mNumOfThreads = numOfThreads;}
  void setUseCUDA(bool useCUDA){

#ifdef USE_CUDA
    mUseCUDA = useCUDA;
#else
    if(useCUDA) {
      std::cerr<<"ObjRecRANSAC::"<<std::string(__func__)<<"(): ObjRecRANSAC wasn't compiled with -DUSE_CUDA, so CUDA mode cannot be enabled."<<std::endl;
    }
#endif
  }

  void setCUDADeviceMap(std::vector<int> &deviceMap){
    boost::recursive_mutex::scoped_lock computing_lock(mComputingMutex);

    mCUDADeviceMap.assign(deviceMap.begin(), deviceMap.end());
  }
  void setDebugNormalRadius(double radius) { mDebugNormalRadius = radius; }
  void setDebugNormals(int num_clouds) { mDebugNormals = num_clouds; }

  vtkPoints* getInputScene(){ return mInputScene;}
  ORROctree* getSceneOctree(){ return mSceneOctree;}
  ORRRangeImage2* getSceneRangeImage(){ return &mSceneRangeImage;}
  ModelDatabase* getModelDatabase(){ return &mModelDatabase;}
  DatabaseModelEntry* getDatabaseModelEntry(vtkPolyData* polydata){ return mModelDatabase.getModelEntry(polydata);}

  double getLastOverallRecognitionTimeSec(){ return mLastOverallRecognitionTimeSec;}
  double getLastPureRecognitionTimeSec(){ return mLastPureRecognitionTimeSec;}
  int getLastNumberOfCheckedHypotheses(){ return mNumOfHypotheses;}
  int getNumberOfThreads(){ return mNumOfThreads;}
  list<OrientedPair>& getSampledPairs(){ return mSampledPairs;}

  void useAbsoluteObjSize(int numOfModelPoints){ mEstOfModelPointsInTheScene = numOfModelPoints; mUseAbsoluteObjSize = true;}
  void useRelativeObjSize(){ mUseAbsoluteObjSize = false;}

  /** This method clears everything (frees memory, etc...). It is called automatically by the destruction
   * of this object. */
  void clear();

  void printParameters(FILE* fp);

protected:
  void sampleOrientedPointPairs(OctreeNode** leaves1, int numOfLeaves, list<OrientedPair>& pairs);
  void generateHypotheses(const list<OrientedPair>& pairs);
  void generateHypothesesForPair(const double* scenePoint1, const double* sceneNormal1,
                                 const double* scenePoint2, const double* sceneNormal2, HashTableCell** cells,
                                 int numOfCells, int pair_id);
  void acceptHypotheses(list<AcceptedHypothesis>& acceptedHypotheses);
  void hypotheses2Shapes(list<AcceptedHypothesis>& hypotheses, vector<boost::shared_ptr<ORRPointSetShape> >& shapes);

  void clear_rec();
  void init_rec(vtkPoints* scene);
  bool fillModelHashTable(vtkPolyData* model);
  void checkObjectHypotheses(const double* p1, const double* n1, const double* p2, const double* n2,
                             HashTableCell** cells, int numOfCells);
  inline void collectCenteredPoints(list<OctreeNode*>& nodes, double** out);
  inline double* estimateNodeNormal(double** ptsToUse, OctreeNode* node, ORROctreeNodeData* node_data);
  void gridBasedFiltering(vector<boost::shared_ptr<ORRPointSetShape> >& shapes, list<boost::shared_ptr<ORRPointSetShape> >& out);
  inline int computeNumberOfIterations(double successProbability, int numOfScenePoints) const;
  inline void estimateSceneNormals();

  /** Computes the intersection between %shape1 and %shape2. Returns true if the intersection is significant,
   * meaning that the cardinality of the intersection set is a large fraction of the id sets of the shapes. */
  inline bool significantIntersection(boost::shared_ptr<ORRPointSetShape>  shape1, boost::shared_ptr<ORRPointSetShape>  shape2);

  inline void getIdPair(const boost::shared_ptr<ORRPointSetShape>  shape1, const boost::shared_ptr<ORRPointSetShape>  shape2, std::pair<int,int>& id_pair);

  inline void cvPolarDecomp(const double M[9], double R[9]);

public:
  list<OrientedPair> mSampledPairs;

  // greedy objrec compatibility
  int getHypotheses(vtkPoints* scene, double successProbability, list<AcceptedHypothesis> &acc_hypotheses);

protected:
  ModelDatabase mModelDatabase;
  ORROctree *mSceneOctree;
  OptimalTransformation mOptTransform;
  PCA mPCA;
  double mLastOverallRecognitionTimeSec, mLastPureRecognitionTimeSec;
  int mNumOfThreads, mNumOfHypotheses;
  vtkPoints* mInputScene;

  ORRRangeImage2 mSceneRangeImage;
  list<list<int>* > mOccupiedPixelsByShapes;
  vector<boost::shared_ptr<ORRPointSetShape> > mShapes;

  bool mUseAbsoluteObjSize;

  std::vector<Hypothesis> mHypotheses;
  std::vector<double> mRigidTransforms;
  std::vector<const double*> mPointSetPointers;
  std::vector<int> mPairIds;
  std::vector<DatabaseModelEntry *> mModelEntryPointers;

  // Parameters
  int mNormalEstimationNeighRadius, mNumOfPointsPerLayer, mEstOfModelPointsInTheScene;
  double mRelNumOfPairsInTheHashTable;
  double mVoxelSize, mVisibility, mRelativeNumOfIllegalPts;
  double mRelativeObjSize, mPairWidth, mAbsZDistThresh;
  double mRelNumOfPairsToKill, mIntersectionFraction;
  bool mICPRefinement;
  double mICPEpsilon;

  // CUDA swtich
  bool mUseCUDA;
  std::vector<int> mCUDADeviceMap;

  int mDebugNormals;
  double mDebugNormalRadius;

  boost::recursive_mutex mComputingMutex;

  // Working structures for optimization
  std::vector<int> mIDs;
  std::vector<OctreeNode*> mLeaves;
  RandomGenerator mRandGen;

  //For collecting profiling averages
  double mHypoGenRate;
  double mProfiling[12];
  int mDoRecognitionCount;
};

//=== inline methods ==============================================================================================

int ObjRecRANSAC::computeNumberOfIterations(double successProbability, int numOfScenePoints) const
{
  // 'p_obj' is the probability that given that the first sample point belongs to an object,
  // the second sample point will belong to the same object
  double p_obj = 0.25;
  double P;

  if ( mUseAbsoluteObjSize )
    P = p_obj*((double)mEstOfModelPointsInTheScene/numOfScenePoints)*mRelNumOfPairsInTheHashTable;
  else
    P = p_obj*mRelativeObjSize*mRelNumOfPairsInTheHashTable;

  if ( 1.0-P <= 0.0 )
    return 1;

  return (int)(log(1.0-successProbability)/log(1.0-P) + 1.0);
}

//=================================================================================================================

inline void ObjRecRANSAC::collectCenteredPoints(list<OctreeNode*>& nodes, double** out)
{
  const double *p;
  double com[3] = {0.0, 0.0, 0.0};
  list<OctreeNode*>::iterator it;

  for ( it = nodes.begin() ; it != nodes.end() ; ++it )
  {
    p = ((ORROctreeNodeData*)(*it)->getData())->getPoint();
    com[0] += p[0];
    com[1] += p[1];
    com[2] += p[2];
  }
  com[0] /= (double)nodes.size();
  com[1] /= (double)nodes.size();
  com[2] /= (double)nodes.size();

  int i;
  for ( i = 0, it = nodes.begin() ; it != nodes.end() ; ++it, ++i )
  {
    p = ((ORROctreeNodeData*)(*it)->getData())->getPoint();
    out[0][i] = p[0] - com[0];
    out[1][i] = p[1] - com[1];
    out[2][i] = p[2] - com[2];
  }
}

//================================================================================================================================

inline double* ObjRecRANSAC::estimateNodeNormal(double** ptsToUse, OctreeNode* node, ORROctreeNodeData* node_data)
{
  if ( !node_data->getNormal() )
  {
    list<OctreeNode*> pca_nodes;
    double eigenvals[3], eigenvecs[3][3];

    mSceneOctree->getFullNeighbours(node, mNormalEstimationNeighRadius, pca_nodes);
    // Check if we have enough nodes for a PCA
    if ( pca_nodes.size() < 4 )
      return NULL;

    // Collect the middle points of the nodes and center them
    this->collectCenteredPoints(pca_nodes, ptsToUse);
    // Perform the real PCA
    mPCA.eigenComputations(ptsToUse, pca_nodes.size(), eigenvecs, eigenvals);
    // Switch the normal if necessary
    if ( eigenvecs[2][2] > 0.0 )
    {
      eigenvecs[0][2] = -eigenvecs[0][2];
      eigenvecs[1][2] = -eigenvecs[1][2];
      eigenvecs[2][2] = -eigenvecs[2][2];
    }
    // Save the normal
    node_data->allocNormal();
    node_data->setNormal(eigenvecs[0][2], eigenvecs[1][2], eigenvecs[2][2]);
  }

  return node_data->getNormal();
}

//================================================================================================================================

inline bool ObjRecRANSAC::significantIntersection(boost::shared_ptr<ORRPointSetShape>  shape1, boost::shared_ptr<ORRPointSetShape>  shape2)
{
  // Some variables for the intersection between both shapes
  vector<int> intersection(shape1->getLinearPixelIds().size() + shape2->getLinearPixelIds().size());
  // Compute the intersection set
  vector<int>::iterator intersection_it = set_intersection(
      shape1->getLinearPixelIds().begin(),
      shape1->getLinearPixelIds().end(),
      shape2->getLinearPixelIds().begin(),
      shape2->getLinearPixelIds().end(), intersection.begin());

  double frac_1 = (double)(intersection_it - intersection.begin())/(double)shape1->getNumberOfOccupiedScenePixels();
  double frac_2 = (double)(intersection_it - intersection.begin())/(double)shape2->getNumberOfOccupiedScenePixels();

#ifdef OBJ_REC_RANSAC_VERBOSE_1
  //	printf("\tintersection fractions = %lf (%s: %i, %.3lf), %lf (%s: %i, %.3lf) ",
  //			frac_1, shape1->getLabel(), shape1->getNumberOfOccupiedScenePixels(), shape1->getScore(),
  //			frac_2, shape2->getLabel(), shape2->getNumberOfOccupiedScenePixels(), shape2->getScore());
#endif

  if ( frac_1 > mIntersectionFraction || frac_2 > mIntersectionFraction )
    return true;

  return false;
}

//================================================================================================================================

inline void ObjRecRANSAC::getIdPair(const boost::shared_ptr<ORRPointSetShape> shape1, const boost::shared_ptr<ORRPointSetShape> shape2, std::pair<int,int>& id_pair)
{
  if ( shape1->getShapeId() <= shape2->getShapeId() )
  {
    id_pair.first = shape1->getShapeId();
    id_pair.second = shape2->getShapeId();
  }
  else
  {
    id_pair.first = shape2->getShapeId();
    id_pair.second = shape1->getShapeId();
  }
}

//================================================================================================================================
#ifdef USE_CUDA
//extern void c_polar_decomposition(const double M[9], double R[9]);
#endif
inline void ObjRecRANSAC::estimateSceneNormals()
{
  printf("ObjRecRANSAC::%s(): estimating scene normals ... ", __func__); fflush(stdout);

  // Compute the maximal number of neighbor leaves:
  int tmp = 2*mNormalEstimationNeighRadius + 1;
  int maxNumOfNeighbours = tmp*tmp*tmp;
  vector<OctreeNode*>& fullLeaves = mSceneOctree->getFullLeafs();
  // Reserve memory for the max number of neighbors
  double** pca_pts = mat_alloc(3, maxNumOfNeighbours);

  // Estimate the normals for each octree leaf
  for ( int i = 0 ; i < (int)fullLeaves.size() ; ++i )
    this->estimateNodeNormal(pca_pts, fullLeaves[i], (ORROctreeNodeData*)fullLeaves[i]->getData());

  // Cleanup
  mat_dealloc(pca_pts, 3);

  printf("done.\n"); fflush(stdout);
}

//============================================================================================================================================

inline void cv_polar_decomposition(const double M[9], double R[9])
{
  cv::Mat mcv(3, 3, CV_64FC1), qcv(3, 3, CV_64FC1);

  mcv.at<double>(0, 0) = M[0]; mcv.at<double>(0, 1) = M[1]; mcv.at<double>(0, 2) = M[2];
  mcv.at<double>(1, 0) = M[3]; mcv.at<double>(1, 1) = M[4]; mcv.at<double>(1, 2) = M[5];
  mcv.at<double>(2, 0) = M[6]; mcv.at<double>(2, 1) = M[7]; mcv.at<double>(2, 2) = M[8];

  cv::SVD svd(mcv);
  qcv = svd.u*svd.vt;

  if ( cv::determinant(qcv) < 0.0 )
  {
    svd.vt.at<double>(2,0) = -svd.vt.at<double>(2,0);
    svd.vt.at<double>(2,1) = -svd.vt.at<double>(2,1);
    svd.vt.at<double>(2,2) = -svd.vt.at<double>(2,2);
    qcv = svd.u*svd.vt;
  }

  // Write back
  R[0] = qcv.at<double>(0, 0); R[1] = qcv.at<double>(0, 1); R[2] = qcv.at<double>(0, 2);
  R[3] = qcv.at<double>(1, 0); R[4] = qcv.at<double>(1, 1); R[5] = qcv.at<double>(1, 2);
  R[6] = qcv.at<double>(2, 0); R[7] = qcv.at<double>(2, 1); R[8] = qcv.at<double>(2, 2);
}

//================================================================================================================================

inline void one_icp_iteration(const double* mp, int numOfPoints, const GeometryProcessor *geom_processor,
                              const ORRRangeImage2* image, double* transform, double* C, double* Ncc, double* m_0, double* s_0)
{
  int k, x, y, match = 0;
  const double_2* pixel;
  const double *sp;
  double out[3];

  // Some initializations
  m_0[0] = m_0[1] = m_0[2] = 0.0;
  s_0[0] = s_0[1] = s_0[2] = 0.0;
  C[0] = C[1] = C[2] = C[3] = C[4] = C[5] = C[6] = C[7] = C[8] = 0.0;

  // The ICP loop
  for ( k = 0 ; k < numOfPoints ; ++k, mp += 3 )
  {
    // Transform the model point with the current rigid transform
    mat_mult3_by_rigid<double>(transform, mp, out);

    // Get the pixel the point 'out' lies in
    pixel = image->getSafePixel(out[0], out[1], x, y);
    // Check if we have a valid pixel
    if ( !pixel )
      continue;

    if ( out[2] < pixel->x ) // The transformed model point overshadows a pixel
      continue;
    else if ( out[2] <= pixel->y ) // The point is OK.
    {
      ++match;

      // Get the scene point
      sp = image->getGridSet(x, y)->p();

      // Contribute to the center of mass
      vec_add3(m_0, mp); // We use 'out' only to establish the correspondence
      vec_add3(s_0, sp);
      // Contribute to the covariance matrix
      mat_add_tensor_product_to_mat9(sp, mp, C);
    }
  }

  // We need at least three corresponding point pairs
  if ( match < 3 )
    return;

  // Compute the center of mass for the model
  vec_mult3(m_0, 1.0/(double)match);
  // Compute 'Ncc'
  mat_tensor_product9(s_0, m_0, Ncc);
  // Compute the covariance matrix
  mat_sub9(C, Ncc);
  // Compute the optimal rotation and save it in 'transform'
  //	ipp_polar_decomposition(C, transform);
  cv_polar_decomposition(C, transform);
  //c_polar_decomposition(C, transform);

  // Compute the center of mass for the scene
  vec_mult3(s_0, 1.0/(double)match);

  // Compute the optimal translation
  transform[9]  = s_0[0] - (transform[0]*m_0[0] + transform[1]*m_0[1] + transform[2]*m_0[2]);
  transform[10] = s_0[1] - (transform[3]*m_0[0] + transform[4]*m_0[1] + transform[5]*m_0[2]);
  transform[11] = s_0[2] - (transform[6]*m_0[0] + transform[7]*m_0[1] + transform[8]*m_0[2]);
}

//================================================================================================================================
#if 0
inline void ipp_polar_decomposition(const double A[9], double R[9])
{
  Ipp64f MMT[9], MTM[9], M[9] = {A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8]};
  mat_mult_MMT_9<Ipp64f>(M, MMT);
  mat_mult_MTM_9<Ipp64f>(M, MTM);

  Ipp64f eigen_vals[3], buffer[9], U[9], V[9];
  int stride1 = 3*sizeof(Ipp64f), stride2 = sizeof(Ipp64f);

  ippmEigenValuesVectorsSym_m_64f(MMT, stride1, stride2, buffer, U, stride1, stride2, eigen_vals, 3);
  ippmEigenValuesVectorsSym_m_64f(MTM, stride1, stride2, buffer, V, stride1, stride2, eigen_vals, 3);

  Ipp64f UTM[9], S_0, S_1, S_2;
  mat_mult_AT_B_9<Ipp64f>(U, M, UTM);

  // These are the diagonal elements of the Sigma matrix
  S_0 = UTM[0]*V[0] + UTM[1]*V[3] + UTM[2]*V[6];
  S_1 = UTM[3]*V[1] + UTM[4]*V[4] + UTM[5]*V[7];
  S_2 = UTM[6]*V[2] + UTM[7]*V[5] + UTM[8]*V[8];

  if ( S_0 < 0.0 )
  {
    // Change the column sign
    U[0] = -U[0]; U[3] = -U[3]; U[6] = -U[6];
  }
  if ( S_1 < 0.0 )
  {
    // Change the column sign
    U[1] = -U[1]; U[4] = -U[4]; U[7] = -U[7];
  }
  if ( S_2 < 0.0 )
  {
    // Change the column sign
    U[2] = -U[2]; U[5] = -U[5]; U[8] = -U[8];
  }

  Ipp64f VT[9], Q[9];

  mat_transpose9<Ipp64f>(V, VT);
  mat_mult9<Ipp64f>(U, VT, Q);

  // Make sure we have a rotation
  if ( mat_determinant9<Ipp64f>(Q) < 0.0 )
  {
    VT[6] = -VT[6];
    VT[7] = -VT[7];
    VT[8] = -VT[8];
    mat_mult9<Ipp64f>(U, VT, Q);
  }

  // Copy to 'R'
  R[0] = Q[0]; R[1] = Q[1]; R[2] = Q[2];
  R[3] = Q[3]; R[4] = Q[4]; R[5] = Q[5];
  R[6] = Q[6]; R[7] = Q[7]; R[8] = Q[8];
}
#endif
//================================================================================================================================

#endif /*OBJRECRANSAC_H_*/
