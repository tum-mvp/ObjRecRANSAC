/*
 * File:   utility.h
 * Author: chi
 *
 * Created on April 4, 2015, 11:13 AM
 */

#ifndef UTILITY_H
#define	UTILITY_H

#include <BasicTools/Vtk/VtkTransform.h>
#include <BasicTools/ComputationalGeometry/Algorithms/RANSACPlaneDetector.h>
#include <ObjRecRANSAC/ObjRecRANSAC.h>
#include <VtkBasics/VtkWindow.h>
#include <VtkBasics/VtkPolyData.h>
#include <VtkBasics/VtkPoints.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkTransformPolyDataFilter.h>
#include <list>
#include <vtkNew.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/usc.h>
#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <boost/iterator/filter_iterator.hpp>
#include <pcl/surface/mls.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/common/norms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <sys/time.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

#include <omp.h>

typedef pcl::PointXYZ myPointXYZ;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::Normal NormalT;

struct ModelT{
  pcl::PointCloud<myPointXYZ>::Ptr model_cloud;
  pcl::PolygonMesh::Ptr model_mesh;
  pcl::PointCloud<myPointXYZ>::Ptr model_center;
  std::string model_label;
};

struct poseT{
  std::string model_name;
  Eigen::Vector3f shift;
  Eigen::Quaternion<float> rotation;
};

int readCSV(std::string filename, std::string label, std::vector<poseT> &poses);
int writeCSV(std::string filename, std::string label, const std::vector<poseT> &poses);
//int readCSV(std::string filename, std::vector< std::vector<float> > &poses);
//int writeCSV(std::string filename, const std::vector< std::vector<float> > &poses);

vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
//vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(const pcl::PointCloud<PointT>::Ptr cloud);
//vtkPoints* PolyDataFromPointCloud(const pcl::PointCloud<myPointXYZ>::Ptr cloud);
//vtkPoints* PolyDataFromPointCloud(const pcl::PointCloud<PointT>::Ptr cloud);

void splitCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<myPointXYZ>::Ptr link_cloud, pcl::PointCloud<myPointXYZ>::Ptr node_cloud);
float sqrDistPt(const myPointXYZ &pt1, const myPointXYZ &pt2);
float sqrDistPtT(const PointT &pt1, const PointT &pt2);
pcl::PointCloud<myPointXYZ>::Ptr FilterCloud(const pcl::PointCloud<myPointXYZ>::Ptr scene, const pcl::PointCloud<myPointXYZ>::Ptr tran_model, float T = 0.015);

bool exists_test (const std::string& name);
bool exists_dir (const std::string& name);
double get_wall_time();
pcl::PointCloud<myPointXYZ>::Ptr ComputeCentroid(pcl::PointCloud<myPointXYZ>::Ptr cloud);

ModelT LoadMesh(std::string filename, std::string label);

#define FOCAL_X 574.0527954101562
#define FOCAL_Y 574.0527954101562
#define CENTER_X 319.5
#define CENTER_Y 239.5
//#define FOCAL_X 570.3
//#define FOCAL_Y 570.3

#endif	/* UTILITY_H */

