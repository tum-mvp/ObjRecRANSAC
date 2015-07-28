/*
 * File:   newmain.cpp
 * Author: chi
 *
 * Created on April 4, 2015, 11:20 AM
 */

#include "src/Greedy/utility.h"
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <opencv2/highgui/highgui.hpp>

std::string link_mesh_name("data/link_uniform.obj");
std::string node_mesh_name("data/node_uniform.obj");

ModelT LoadMesh(std::string filename, std::string label);
void showGT(const std::vector<ModelT> &model_set, const std::vector<poseT> &poses, pcl::visualization::PCLVisualizer::Ptr viewer);
//int MeshOn2D(const std::vector<ModelT> &model_set, const std::vector<poseT> &poses, cv::Mat &map2d, float fx = FOCAL_X, float fy = FOCAL_Y);
float segAcc(const std::vector<ModelT> &model_set, const std::vector<poseT> &poses, pcl::PointCloud<PointT>::Ptr cloud)
{
  pcl::PointCloud<PointT>::Ptr link_cloud(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr node_cloud(new pcl::PointCloud<PointT>());

  for( int i = 0 ; i < model_set.size() ; i++ )
  {
    uint32_t cur_label;
    pcl::PointCloud<PointT>::Ptr cur_cloud(new pcl::PointCloud<PointT>());
    if( model_set[i].model_label == "link" )
    {
      cur_label = 1;
      cur_cloud = link_cloud;
    }
    else if( model_set[i].model_label == "node" )
    {
      cur_label = 2;
      cur_cloud = node_cloud;
    }
    pcl::copyPointCloud(*model_set[i].model_cloud, *cur_cloud);
    for( pcl::PointCloud<PointT>::iterator it = cur_cloud->begin() ; it < cur_cloud->end() ; it++ )
      it->rgba = cur_label;
  }

  Eigen::Quaternionf calibrate_rot(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f (1, 0, 0)));
  pcl::PointCloud<PointT>::Ptr all_cloud(new pcl::PointCloud<PointT>());
  for(std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++)
  {
    for( int i = 0 ; i < model_set.size() ; i++ )
    {
      if(model_set[i].model_label == it->model_name )
      {
        pcl::PointCloud<PointT>::Ptr cur_cloud(new pcl::PointCloud<PointT>());
        if( it->model_name == "link" )
          pcl::copyPointCloud(*link_cloud, *cur_cloud);
        else if( it->model_name == "node" )
          pcl::copyPointCloud(*node_cloud, *cur_cloud);

        pcl::transformPointCloud(*cur_cloud, *cur_cloud, it->shift, it->rotation*calibrate_rot);

        all_cloud->insert(all_cloud->end(), cur_cloud->begin(), cur_cloud->end());
      }
    }
  }

  pcl::search::KdTree<PointT> tree;
  tree.setInputCloud (all_cloud);

  uint8_t tmp_color = 255;
  uint32_t red = tmp_color << 16;
  uint32_t blue = tmp_color;

  int pos_count = 0;
  float T = 0.02*0.02;
  for ( pcl::PointCloud<PointT>::iterator it = cloud->begin() ; it < cloud->end() ; it++ )
  {
    std::vector<int> indices (1);
    std::vector<float> sqr_distances (1);
    int nres = tree.nearestKSearch(*it, 1, indices, sqr_distances);
    if( it->rgba > 255 )
      it->rgba = 1;
    else if( it->rgba > 0 )
      it->rgba = 2;
    else
      it->rgba = 0;
    if ( nres == 1 && sqr_distances[0] < T )
    {
      if(it->rgba == all_cloud->at(indices[0]).rgba)
        pos_count++;
    }
    else if( it->rgba == 0 || sqr_distances[0] > T )
      pos_count++;

    if( nres == 1 && sqr_distances[0] < T )
    {
      if( all_cloud->at(indices[0]).rgba == 1 )
        it->rgba = red;
      else if( all_cloud->at(indices[0]).rgba == 2 )
        it->rgba = blue;
    }
    else
      it->rgba = 0;
  }

  return (pos_count +0.0) / cloud->size();
}


int main(int argc, char** argv)
{
  std::string seg_path("data/ln_joint/");
  std::string gt_path("final_joint_gt/");
  std::string out_path("");

  //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
  //viewer->initCameraParameters();
  //viewer->addCoordinateSystem(0.1);
  //viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);

  int c1 = 0, c2 = -1;
  pcl::console::parse_argument(argc, argv, "--gt", gt_path);
  pcl::console::parse_argument(argc, argv, "--eg", seg_path);
  pcl::console::parse_argument(argc, argv, "--o", out_path);

  pcl::console::parse_argument(argc, argv, "--c1", c1);
  pcl::console::parse_argument(argc, argv, "--c2", c2);

  if( out_path.empty() == false && exists_dir(out_path) == false )
    boost::filesystem::create_directories(out_path);

  float acc = 0;
  int acc_count = 0;
  for(int i = c1 ; i <= c2 ; i++ )
  {
    std::stringstream ss;
    ss << i;

    std::string pcd_file(seg_path + "seg_" + ss.str() + ".pcd");
    std::string link_pose_file(gt_path + "link_gt_" + ss.str() + ".csv");
    std::string node_pose_file(gt_path + "node_gt_" + ss.str() + ".csv");
    std::cerr << "Loading... " << pcd_file << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(pcd_file, *cloud);
    if( cloud->empty() == true )
      break;
    acc_count++;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<poseT> all_poses;
    readCSV(link_pose_file, "link", all_poses);
    readCSV(node_pose_file, "node", all_poses);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //read meshes
    ModelT link_mesh = LoadMesh(link_mesh_name, "link");
    ModelT node_mesh = LoadMesh(node_mesh_name, "node");

    std::vector<ModelT> mesh_set;
    mesh_set.push_back(link_mesh);
    mesh_set.push_back(node_mesh);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //viewer->removePointCloud("cloud");
    //viewer->addPointCloud(cloud, "cloud");
    //viewer->spin();

    float cur_acc = segAcc(mesh_set, all_poses, cloud);
    std::cerr << "P: " << cur_acc << std::endl;
    acc += cur_acc;

    if( out_path.empty() == false )
      pcl::io::savePCDFile(out_path + "seg_" + ss.str() + ".pcd", *cloud, true);

    //viewer->removePointCloud("cloud");
    //viewer->addPointCloud(cloud, "cloud");
    //viewer->spin();
    /*
       cv::Mat gt_label, seg_label, uv;
       int gt_num = MeshOn2D(mesh_set, all_poses, gt_label);
       int seg_num = segProj(cloud, seg_label, uv);
       int true_pos = overlapGT(gt_label, seg_label, uv);

       std::cerr << "P: " << (true_pos+0.0) / seg_num << std::endl;
       acc += (true_pos+0.0) / seg_num;
    //*
    showLabel(gt_label, "GT");
    showLabel(seg_label, "Seg");
    //*/
  }
  std::cerr << "Seg-Acc: " << acc / acc_count << std::endl;

  /*************************************************************************************************************************/

  return 0;
}

void showGT(const std::vector<ModelT> &model_set, const std::vector<poseT> &poses, pcl::visualization::PCLVisualizer::Ptr viewer)
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
        pcl::transformPointCloud(*rec[i], *cur_cloud, it->shift, it->rotation*calibrate_rot);

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

/*
   int MeshOn2D(const std::vector<ModelT> &model_set, const std::vector<poseT> &poses, cv::Mat &map2d, float fx, float fy)
   {
   int img_h = 480;
   int img_w = 640;
   cv::Mat elev_map = cv::Mat::ones(img_h, img_w, CV_32FC1)*1000;
   map2d = cv::Mat::zeros(img_h, img_w, CV_32SC1);

   int count = 0;
   Eigen::Quaternionf calibrate_rot(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f (1, 0, 0)));
   for(std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++)
   {
   for( int i = 0 ; i < model_set.size() ; i++ )
   {
   if(model_set[i].model_label == it->model_name )
   {
   pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>());
   pcl::transformPointCloud(*model_set[i].model_cloud, *cur_cloud, Eigen::Vector3f (0, 0, 0), calibrate_rot);
   pcl::transformPointCloud(*cur_cloud, *cur_cloud, it->shift, it->rotation);

   int cur_label;
   if( it->model_name == "link" )
   cur_label = 1;
   else if( it->model_name == "node" )
   cur_label = 2;

   size_t num = cur_cloud->size();
   for( size_t i = 0 ; i < num ; i++ )
   {
   myPointXYZ *pt_ptr = &cur_cloud->at(i);
   int img_x = round(pt_ptr->x / pt_ptr->z * fx + CENTER_X);
   int img_y = round(pt_ptr->y / pt_ptr->z * fy + CENTER_Y);
   if( img_x < 0 ) img_x = 0;
   if( img_y < 0 ) img_y = 0;
   if( img_x >= img_w ) img_x = img_w-1;
   if( img_y >= img_h ) img_y = img_h-1;

   if( pt_ptr->z < elev_map.at<float>(img_y, img_x) )
   {
   if( map2d.at<int>(img_y, img_x) <= 0 )
   count++;
   map2d.at<int>(img_y, img_x) = cur_label;
   elev_map.at<float>(img_y, img_x) = pt_ptr->z;
   }
   }
   }
   }
   }

   return count;
   }


   int segProj(const pcl::PointCloud<PointT>::Ptr cloud, cv::Mat &map2d, cv::Mat &uv, float fx = FOCAL_X, float fy = FOCAL_Y)
   {
   size_t num = cloud->size();
   int img_h = 480;
   int img_w = 640;

   map2d = cv::Mat::ones(img_h, img_w, CV_32SC1)*-1;
   uv = cv::Mat::zeros(num, 2, CV_32SC1);
   int *ptr_uv = (int *)uv.data;

   cv::Mat link_count = cv::Mat::zeros(img_h, img_w, CV_32SC1);
   cv::Mat node_count = cv::Mat::zeros(img_h, img_w, CV_32SC1);
   cv::Mat back_count = cv::Mat::zeros(img_h, img_w, CV_32SC1);

   int count = 0;
   for( size_t i = 0 ; i < num ; i++ )
   {
   PointT *pt_ptr = &cloud->at(i);
int img_x = round(pt_ptr->x / pt_ptr->z * fx + CENTER_X);
int img_y = round(pt_ptr->y / pt_ptr->z * fy + CENTER_Y);
if( img_x < 0 ) img_x = 0;
if( img_y < 0 ) img_y = 0;
if( img_x >= img_w ) img_x = img_w-1;
if( img_y >= img_h ) img_y = img_h-1;

if( map2d.at<int>(img_y, img_x) < 0 )
{
  *ptr_uv = img_y; ptr_uv++;
  *ptr_uv = img_x; ptr_uv++;
  count++;
}
if( pt_ptr->rgba > 255 )
  link_count.at<int>(img_y, img_x)++;
else if( pt_ptr->rgba > 0 )
  node_count.at<int>(img_y, img_x)++;
  else
  back_count.at<int>(img_y, img_x)++;

  int link_num = link_count.at<int>(img_y, img_x);
  int node_num = node_count.at<int>(img_y, img_x);
  int back_num = back_count.at<int>(img_y, img_x);
if( link_num > node_num && link_num > back_num )
  map2d.at<int>(img_y, img_x) = 1;
else if( node_num > back_num )
  map2d.at<int>(img_y, img_x) = 2;
  else
  map2d.at<int>(img_y, img_x) = 0;
  }

return count;
}

int overlapGT(const cv::Mat &gt_label, const cv::Mat &seg_label, const cv::Mat &uv)
{
  int num = uv.rows;
  int count = 0;
  cv::Mat diff = cv::Mat::zeros(480, 640, CV_8UC1);
  for(int i = 0 ; i < num ; i++ )
  {
    int img_y = uv.at<int>(i, 0);
    int img_x = uv.at<int>(i, 1);

    if( gt_label.at<int>(img_y, img_x) == seg_label.at<int>(img_y, img_x) )
      count++;
    if( gt_label.at<int>(img_y, img_x) == 0 && seg_label.at<int>(img_y, img_x) > 0)
      count++;
    //diff.at<uchar>(img_y, img_x) = 255;
  }
  //cv::imshow("DIFF", diff);
  //cv::waitKey();
  return count;
}

void showLabel(cv::Mat label_map, std::string name)
{
  cv::Mat show_gt = cv::Mat::zeros(label_map.rows, label_map.cols, CV_8UC3);
  for(int r = 0 ; r < label_map.rows; r++ ){
    for(int c = 0 ; c < label_map.cols; c++ )
    {
      if(label_map.at<int>(r, c) == 1 )
      {
        show_gt.at<uchar>(r, c*3+2) = 255;
        show_gt.at<uchar>(r, c*3+1) = 0;
        show_gt.at<uchar>(r, c*3) = 0;
      }
      else if(label_map.at<int>(r, c) == 2 )
      {
        show_gt.at<uchar>(r, c*3+2) = 0;
        show_gt.at<uchar>(r, c*3+1) = 0;
        show_gt.at<uchar>(r, c*3) = 255;
      }
    }
  }
  cv::imshow(name.c_str(), show_gt);
  cv::waitKey();
}
*/
