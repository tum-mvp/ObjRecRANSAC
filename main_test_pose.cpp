/* 
 * File:   newmain.cpp
 * Author: chi
 *
 * Created on April 4, 2015, 11:20 AM
 */

#include "src/Greedy/utility.h"
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

std::string link_mesh_name("data/link_uniform.obj");
std::string node_mesh_name("data/node_uniform.obj");

void showPoses(const std::vector<ModelT> &model_set, const std::vector<poseT> &poses, pcl::visualization::PCLVisualizer::Ptr viewer, bool adjust = true)
{
  std::vector<pcl::PointCloud<myPointXYZ>::Ptr> rec;
  for( std::vector<ModelT>::const_iterator it = model_set.begin() ; it < model_set.end() ; it++ )
  {
    pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
    pcl::fromPCLPointCloud2(it->model_mesh->cloud, *cur_cloud);
    rec.push_back(cur_cloud);
  }

  int count = 0;
  Eigen::Quaternionf calibrate_rot(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f (1, 0, 0)));
  for(std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++, count++ )
  {
    for( int i = 0 ; i < model_set.size() ; i++ )
    {
      if(model_set[i].model_label == it->model_name )
      {
        pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
        //pcl::transformPointCloud(*rec[i], *cur_cloud, Eigen::Vector3f (0, 0, 0), calibrate_rot);
        if( adjust )
          pcl::transformPointCloud(*rec[i], *cur_cloud, it->shift, it->rotation*calibrate_rot);
        else
          pcl::transformPointCloud(*rec[i], *cur_cloud, it->shift, it->rotation);

        std::stringstream ss;
        ss << count;

        viewer->addPolygonMesh<myPointXYZ>(cur_cloud, model_set[i].model_mesh->polygons, it->model_name+"_"+ss.str());
        if( it->model_name == "link" )
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.55, 0.05, it->model_name+"_"+ss.str());
        else if( it->model_name == "node" )
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.05, 0.55, 1.0, it->model_name+"_"+ss.str());
        break;
      }
    }

  }
}

bool matchCloud(const ModelT &model, const ModelT &cloud, float ratio, float T)
{
  if( model.model_label != cloud.model_label || sqrDistPt(model.model_center->at(0), cloud.model_center->at(0)) >= 0.1*0.1 )
    return false;

  pcl::search::KdTree<myPointXYZ> tree;
  tree.setInputCloud(model.model_cloud);

  float sqrT = T*T;
  int count = 0;  
  for( pcl::PointCloud<myPointXYZ>::const_iterator it = cloud.model_cloud->begin() ; it < cloud.model_cloud->end() ; it++ )
  {               
    std::vector<int> indices (1);
    std::vector<float> sqr_distances (1);
    int nres = tree.nearestKSearch(*it, 1, indices, sqr_distances);
    if ( nres >= 1 && sqr_distances[0] <= sqrT )
      count++;
  }
  //std::cerr << "COUNT: " << (count+0.0) / cloud.model_cloud->size() << std::endl;
  if( (count+0.0) / cloud.model_cloud->size() > ratio )
    return true;
  else
    return false;
}

int overlapPose(const std::vector<ModelT> &model_set, const std::vector<poseT> &est_poses, const std::vector<poseT> &gt_poses)
{
  std::vector<ModelT> est_insts, gt_insts;
  for( std::vector<poseT>::const_iterator it = est_poses.begin() ; it < est_poses.end() ; it++ ){
    for( int i = 0 ; i < model_set.size() ; i++ ){
      if( model_set[i].model_label == it->model_name )
      {
        ModelT new_data;
        new_data.model_label = it->model_name;
        new_data.model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); 
        new_data.model_center = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*model_set[i].model_cloud, *new_data.model_cloud, it->shift, it->rotation);
        pcl::transformPointCloud(*model_set[i].model_center, *new_data.model_center, it->shift, it->rotation);
        est_insts.push_back(new_data);
        break;
      }
    }
  }
  Eigen::Quaternionf calibrate_rot(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f (1, 0, 0)));
  for( std::vector<poseT>::const_iterator it = gt_poses.begin() ; it < gt_poses.end() ; it++ ){
    for( int i = 0 ; i < model_set.size() ; i++ ){
      if( model_set[i].model_label == it->model_name )
      {
        ModelT new_data;
        new_data.model_label = it->model_name;
        new_data.model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); 
        new_data.model_center = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*model_set[i].model_cloud, *new_data.model_cloud, it->shift, it->rotation*calibrate_rot);
        pcl::transformPointCloud(*model_set[i].model_center, *new_data.model_center, it->shift, it->rotation*calibrate_rot);
        gt_insts.push_back(new_data);
        break;
      }
    }
  }
  //std::cerr << est_insts.size() << " " << gt_insts.size() << std::endl;
  int true_count = 0;
  for( std::vector<ModelT>::iterator est_it = est_insts.begin(); est_it < est_insts.end() ; est_it++ ){
    for( std::vector<ModelT>::iterator gt_it = gt_insts.begin(); gt_it < gt_insts.end() ; gt_it++ ){
      if( matchCloud(*gt_it, *est_it, 0.7, 0.01) == true )
      {
        true_count++;
        break;
      }
    }
  }

  return true_count;
}

int main(int argc, char** argv) 
{
  //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
  //viewer->initCameraParameters();
  //viewer->addCoordinateSystem(0.1);
  //viewer->setSize(500, 400);
  //viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);

  std::string seq_id("0");
  pcl::console::parse_argument(argc, argv, "--s", seq_id);

  std::string gt_path("final_joint_gt/");
  std::string est_path("result/"+seq_id+"/");

  int c1 = 0, c2 = -1;
  int t1 = 0, t2 = -1;
  pcl::console::parse_argument(argc, argv, "--c1", c1);
  pcl::console::parse_argument(argc, argv, "--c2", c2);
  pcl::console::parse_argument(argc, argv, "--t1", t1);
  pcl::console::parse_argument(argc, argv, "--t2", t2);

  //read meshes
  ModelT link_mesh = LoadMesh(link_mesh_name, "link");
  ModelT node_mesh = LoadMesh(node_mesh_name, "node"); 
  std::vector<ModelT> mesh_set;
  mesh_set.push_back(link_mesh);
  mesh_set.push_back(node_mesh);

  std::vector<float> p_vec(t2-t1+1);
  std::vector<float> r_vec(t2-t1+1);
  for( int t = t1 ; t <= t2 ; t++ )
  {
    std::stringstream tt;
    tt << t;

    int est_total = 0;
    int gt_total = 0;
    int true_total = 0;

    std::cerr << "Loading Trial-" << t << std::endl;
    std::string cur_path(est_path + "tr" + tt.str() + "/");
    for(int i = c1 ; i <= c2 ; i++ )
    {
      std::stringstream ss;
      ss << i;

      std::string link_gt_file(gt_path + "link_gt_" + ss.str() + ".csv");
      std::string node_gt_file(gt_path + "node_gt_" + ss.str() + ".csv");
      std::string link_est_file(cur_path + "link_pose_seg_" + ss.str() + ".csv");
      std::string node_est_file(cur_path + "node_pose_seg_" + ss.str() + ".csv");

      if( exists_test(link_est_file) == false )
      {
        std::cerr << "Failed to Read CSV File!!!" << std::endl;
        exit(0);
      }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      std::vector<poseT> gt_poses;
      readCSV(link_gt_file, "link", gt_poses);
      readCSV(node_gt_file, "node", gt_poses);
      std::vector<poseT> est_poses;
      readCSV(link_est_file, "link", est_poses);
      readCSV(node_est_file, "node", est_poses);
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////        
      //viewer->removeAllPointClouds();
      //showPoses(mesh_set, est_poses, viewer, false);
      //showPoses(mesh_set, gt_poses, viewer, true);
      //viewer->spinOnce(1);
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      int true_pose = overlapPose(mesh_set, est_poses, gt_poses);
      est_total += est_poses.size();
      gt_total += gt_poses.size();
      true_total += true_pose;
      //std::cerr << "P-" << (true_pose+0.0) / est_poses.size() << ",\tR-" << (true_pose+0.0) / gt_poses.size() << std::endl;
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //viewer->spin();
    }
    p_vec[t-t1] = (true_total+0.0) / est_total;
    r_vec[t-t1] = (true_total+0.0) / gt_total;
  }
  std::cerr << "*********************************************************************" << std::endl;
  float avg_p=0, avg_r=0, std_p=0, std_r=0, avg_f = 0, std_f = 0;
  for(int i = 0 ; i < p_vec.size() ; i++ )
  {
    avg_p += p_vec[i];
    std_p += p_vec[i]*p_vec[i];

    avg_r += r_vec[i];
    std_r += r_vec[i]*r_vec[i];

    float cur_f = 2*(p_vec[i]*r_vec[i]) / (p_vec[i]+r_vec[i]);
    avg_f += cur_f;
    std_f += cur_f*cur_f;
  }
  avg_p /= p_vec.size();
  std_p = sqrt(std_p/p_vec.size() - avg_p*avg_p);

  avg_r /= r_vec.size();
  std_r = sqrt(std_r/r_vec.size() - avg_r*avg_r);

  avg_f /= p_vec.size();
  std_f = sqrt(std_f/p_vec.size() - avg_f*avg_f);

  std::cerr << "P-" << avg_p << " +- " << std_p << ",\tR-" << avg_r << " +- " << std_r << ",\tF-" << avg_f << " +- " << std_f << std::endl;
  /*************************************************************************************************************************/

  return 0;
}

