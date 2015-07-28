/*
 * File:   seg.h
 * Author: chi
 *
 * Created on March 30, 2015, 4:56 PM
 */

#include "utility.h"

struct segT{
  pcl::PointCloud<PointT>::Ptr cloud;
  std::vector<int> indices;   //index in original cloud
};

static bool mycomp(const PointT &p1, const PointT &p2)
{
  return p1.z <= p2.z;
}

class greedyObjRansac : public ObjRecRANSAC {
public:
  greedyObjRansac(double pairWidth, double voxelSize = 0.03);
  ~greedyObjRansac(){}

  void AddModel(std::string name, std::string label);
  void visualize(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses);

  void GreedyRecognize(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses);
  void StandardRecognize(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses);

  void genHypotheses(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, list<AcceptedHypothesis> &acc_hypotheses);
  void mergeHypotheses(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, list<AcceptedHypothesis> &acc_hypotheses, std::vector<poseT> &poses);
  pcl::PointCloud<myPointXYZ>::Ptr FillModelCloud(const std::vector<poseT> &poses);
  //void newRecognize( pcl::PointCloud<PointT>::Ptr scene, std::vector<poseMessage> &poses, float confT, pcl::visualization::PCLVisualizer::Ptr viewer);

private:
  std::vector<ModelT> models;
  //ObjRecRANSAC objrec;

  poseT recognizeOne(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, pcl::PointCloud<myPointXYZ>::Ptr &rest_cloud);
  poseT getBestModel(list<boost::shared_ptr<PointSetShape> >& detectedShapes);

  //void genPairFeas(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, list<ObjRecRANSAC::OrientedPair> &PairFeas, float link_width = 0.07, float node_width = 0.04);
  void getPairFeas(const pcl::PointCloud<myPointXYZ>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, list<ObjRecRANSAC::OrientedPair> &PairFeas, float maxDist, int num);

  /*
     pcl::PointCloud<PointT>::Ptr  FilterCloud(const pcl::PointCloud<PointT>::Ptr scene, const std::vector<poseMessage> &poses, float T = 0.015);
     void getAllPairFeas(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, list<ObjRecRANSAC::OrientedPair> &PairFeas, float maxDist);
     void sortCloud(pcl::PointCloud<PointT>::Ptr cloud);
     segT regionGrowing(pcl::PointCloud<PointT>::Ptr cloud, float confT = 0.5);
     */
  double successProbability;      //0.99
  double voxelSize;               //0.03
  double pairWidth;

  double visibility;              //0.1
  double relativeObjSize;         //0.1
};


