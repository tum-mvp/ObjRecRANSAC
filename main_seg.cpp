#include "src/Greedy/seg.h"
#include <sys/time.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <bits/algorithmfwd.h>

//=========================================================================================================================

std::string link_mesh_name("data/link_uniform");
std::string node_mesh_name("data/node_uniform");

//std::string link_mesh_name("data/driller_uniform");
//std::string node_mesh_name("data/sander_uniform");

std::vector<poseT> RefinePoses(const pcl::PointCloud<myPointXYZ>::Ptr scene,
                               const std::vector<ModelT> &mesh_set, const std::vector<poseT> &all_poses);
//=========================================================================================================================

int main(int argc, char** argv)
{
  double linkWidth = 0.15;
  double nodeWidth = 0.05;
  double voxelSize = 0.003;

  int method_id = 4;
  int c1 = 0, c2 = -1;
  std::string trial_id("0");
  std::string path("data/ln_joint/");

  pcl::console::parse_argument(argc, argv, "--m", method_id);
  pcl::console::parse_argument(argc, argv, "--p", path);
  pcl::console::parse_argument(argc, argv, "--t", trial_id);
  pcl::console::parse_argument(argc, argv, "--c1", c1);
  pcl::console::parse_argument(argc, argv, "--c2", c2);

  bool view_flag = false;
  if( pcl::console::find_switch(argc, argv, "-v") == true )
    view_flag = true;
  pcl::visualization::PCLVisualizer::Ptr viewer;
  if( view_flag == true )
  {
    viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setSize(640, 480);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
  }
  greedyObjRansac linkrec(linkWidth, voxelSize);
  greedyObjRansac noderec(nodeWidth, voxelSize);
  greedyObjRansac objrec(nodeWidth, voxelSize);

  std::stringstream mm;
  mm << method_id;

  ModelT link_mesh, node_mesh;
  std::vector<ModelT> mesh_set;
  if( method_id >= 3 )
  {
    link_mesh = LoadMesh(link_mesh_name + ".obj", "link");
    node_mesh = LoadMesh(node_mesh_name + ".obj", "node");
    mesh_set.push_back(link_mesh);
    mesh_set.push_back(node_mesh);
  }

  switch(method_id)
  {
    case 0:
    case 1:
    case 2:
      objrec.AddModel(link_mesh_name, "link");
      objrec.AddModel(node_mesh_name, "node");
      break;
    case 3:
    case 4:
    case 5:
      linkrec.AddModel(link_mesh_name, "link");
      noderec.AddModel(node_mesh_name, "node");
      break;
    default:return 0;
  }

  double avg_t = 0, std_t = 0;
  double t1, t2;
  for( int i = c1 ; i <= c2 ; i++ )
  {
    std::cerr << "Frame-" << i << std::endl;

    std::stringstream ii;
    ii << i;

    std::string filename("seg_"+ii.str()+".pcd");
    std::string scene_name(path+filename);
    pcl::PointCloud<PointT>::Ptr scene_pcd(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(scene_name, *scene_pcd);
    if( scene_pcd->empty() == true )
      break;

    pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
    pcl::copyPointCloud(*scene_pcd, *scene_xyz);
    if( view_flag == true )
    {
      viewer->removeAllPointClouds();
      viewer->addPointCloud(scene_xyz, "whole_scene");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "whole_scene");
    }
    std::vector<poseT> all_poses;
    switch(method_id)
    {
      case 0:
        t1 = get_wall_time();
        objrec.StandardRecognize(scene_xyz, all_poses);
        t2 = get_wall_time();
        break;
      case 1:
        {
          t1 = get_wall_time();
          int pose_num = 0;
          int iter = 0;
          pcl::PointCloud<myPointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<myPointXYZ>());
          filtered_cloud = scene_xyz;
          while(true)
          {
            //std::cerr<< "Recognizing Attempt --- " << iter << std::endl;
            objrec.StandardRecognize(filtered_cloud, all_poses);
            if( all_poses.size() <= pose_num )
              break;
            else
              pose_num = all_poses.size();

            pcl::PointCloud<myPointXYZ>::Ptr model_cloud = objrec.FillModelCloud(all_poses);
            filtered_cloud = FilterCloud(filtered_cloud, model_cloud);

            iter++;
          }
          t2 = get_wall_time();
          //std::cerr<< "Recognizing Done!!!" << std::endl;
          break;
        }
      case 2:
        {
          //std::vector<poseT> tmp_poses;
          t1 = get_wall_time();
          objrec.GreedyRecognize(scene_xyz, all_poses);
          t2 = get_wall_time();
          //all_poses = RefinePoses(scene_xyz, mesh_set, tmp_poses);
          break;
        }
      case 3:
        {
          pcl::PointCloud<myPointXYZ>::Ptr link_cloud(new pcl::PointCloud<myPointXYZ>());
          pcl::PointCloud<myPointXYZ>::Ptr node_cloud(new pcl::PointCloud<myPointXYZ>());
          splitCloud(scene_pcd, link_cloud, node_cloud);

          t1 = get_wall_time();
          std::vector<poseT> link_poses, node_poses;
          linkrec.StandardRecognize(link_cloud, link_poses);
          noderec.StandardRecognize(node_cloud, node_poses);
          t2 = get_wall_time();

          all_poses.insert(all_poses.end(), link_poses.begin(), link_poses.end());
          all_poses.insert(all_poses.end(), node_poses.begin(), node_poses.end());
          break;
        }
      case 4:
        {
          pcl::PointCloud<myPointXYZ>::Ptr link_cloud(new pcl::PointCloud<myPointXYZ>());
          pcl::PointCloud<myPointXYZ>::Ptr node_cloud(new pcl::PointCloud<myPointXYZ>());
          splitCloud(scene_pcd, link_cloud, node_cloud);

          t1 = get_wall_time();
          int pose_num = 0;
          std::vector<poseT> tmp_poses;
          int iter = 0;
          while(true)
          {
            //std::cerr<< "Recognizing Attempt --- " << iter << std::endl;
            list<AcceptedHypothesis> acc_hypotheses;

            linkrec.genHypotheses(link_cloud, acc_hypotheses);
            noderec.genHypotheses(node_cloud, acc_hypotheses);

            linkrec.mergeHypotheses(scene_xyz, acc_hypotheses, tmp_poses);

            if( tmp_poses.size() <= pose_num )
              break;
            else
              pose_num = tmp_poses.size();

            pcl::PointCloud<myPointXYZ>::Ptr link_model = linkrec.FillModelCloud(tmp_poses);
            pcl::PointCloud<myPointXYZ>::Ptr node_model = noderec.FillModelCloud(tmp_poses);

            if( link_model->empty() == false )
              link_cloud = FilterCloud(link_cloud, link_model);
            if( node_model->empty() == false)
              node_cloud = FilterCloud(node_cloud, node_model);
            iter++;
          }
          t2 = get_wall_time();
          //std::cerr<< "Recognizing Done!!!" << std::endl;
          all_poses = RefinePoses(scene_xyz, mesh_set, tmp_poses);
          break;
        }
      case 5:
        {
          pcl::PointCloud<myPointXYZ>::Ptr link_cloud(new pcl::PointCloud<myPointXYZ>());
          pcl::PointCloud<myPointXYZ>::Ptr node_cloud(new pcl::PointCloud<myPointXYZ>());
          splitCloud(scene_pcd, link_cloud, node_cloud);

          t1 = get_wall_time();
          std::vector<poseT> link_poses, node_poses;
          linkrec.GreedyRecognize(link_cloud, link_poses);
          noderec.GreedyRecognize(node_cloud, node_poses);
          t2 = get_wall_time();

          std::vector<poseT> tmp_poses;
          tmp_poses.insert(tmp_poses.end(), link_poses.begin(), link_poses.end());
          tmp_poses.insert(tmp_poses.end(), node_poses.begin(), node_poses.end());

          //all_poses = tmp_poses;
          all_poses = RefinePoses(scene_xyz, mesh_set, tmp_poses);
          break;
        }
      default:return 0;
    }
    avg_t += t2 - t1;
    std_t += (t2 - t1)*(t2 - t1);

    std::string data_name(filename.substr(0, filename.size()-4));
    std::string out_path("result/" + mm.str() + "/tr" + trial_id + "/");
    if( exists_dir(out_path) == false )
      boost::filesystem::create_directories(out_path);

    writeCSV(out_path + "link_pose_" + data_name + ".csv", "link", all_poses);
    writeCSV(out_path + "node_pose_" + data_name + ".csv", "node", all_poses);

    if( view_flag == true )
    {
      switch(method_id)
      {
        case 0:
        case 1:
        case 2:
          objrec.visualize(viewer, all_poses);
          break;
        case 3:
        case 4:
        case 5:
          linkrec.visualize(viewer, all_poses);
          noderec.visualize(viewer, all_poses);
          break;
        default:return 0;
      }
      viewer->saveScreenshot(out_path + data_name + ".png");
      viewer->spin();
    }
  }

  avg_t /= (c2 - c1 + 1);
  std_t = sqrt(std_t/(c2 - c1 + 1) - avg_t*avg_t);

  std::cerr << "Method-" << method_id <<",\tTime-" << avg_t << " +- " << std_t << std::endl;

  return 0;
}

std::vector<poseT> RefinePoses(const pcl::PointCloud<myPointXYZ>::Ptr scene, const std::vector<ModelT> &mesh_set, const std::vector<poseT> &all_poses)
{
  int pose_num = all_poses.size();
  std::vector<ModelT> est_models(pose_num);
  pcl::PointCloud<myPointXYZ>::Ptr down_scene(new pcl::PointCloud<myPointXYZ>());
  pcl::VoxelGrid<myPointXYZ> sor;
  sor.setInputCloud(scene);
  sor.setLeafSize(0.005, 0.005, 0.005);
  sor.filter(*down_scene);

#pragma omp parallel for schedule(dynamic, 1)
  for(int i = 0 ; i < pose_num ; i++ ){
    for( int j = 0 ; j < mesh_set.size() ; j++ ){
      if( mesh_set[j].model_label == all_poses[i].model_name )
      {
        est_models[i].model_label = all_poses[i].model_name;
        est_models[i].model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*mesh_set[j].model_cloud, *est_models[i].model_cloud, all_poses[i].shift, all_poses[i].rotation);
        break;
      }
    }
  }

  std::vector< pcl::search::KdTree<myPointXYZ>::Ptr > tree_set(est_models.size());
#pragma omp parallel for schedule(dynamic, 1)
  for( int i = 0 ; i < pose_num ; i++ )
  {
    tree_set[i] = pcl::search::KdTree<myPointXYZ>::Ptr (new pcl::search::KdTree<myPointXYZ>());
    tree_set[i]->setInputCloud(est_models[i].model_cloud);
  }

  std::vector<int> votes(pose_num, 0);
  std::vector< std::vector<int> > adj_graph(pose_num);
  for( int i = 0 ; i < pose_num ; i++ )
    adj_graph[i].resize(pose_num, 0);
  float sqrT = 0.01*0.01;
  int down_num = down_scene->size();

  std::vector< std::vector<int> > bin_vec(down_num);
#pragma omp parallel for
  for(int i = 0 ; i < pose_num ; i++ )
  {
    int count = 0;
    for( pcl::PointCloud<myPointXYZ>::const_iterator it = down_scene->begin() ; it < down_scene->end() ; it++, count++ )
    {
      std::vector<int> idx (1);
      std::vector<float> sqrDist (1);
      int nres = tree_set[i]->nearestKSearch(*it, 1, idx, sqrDist);
      if ( nres >= 1 && sqrDist[0] <= sqrT )
      {
#pragma omp critical
        {
          bin_vec[count].push_back(i);
        }
        votes[i]++;
      }
    }
  }

  for( int it = 0 ; it < down_num ; it++ )
    for( std::vector<int>::iterator ii = bin_vec[it].begin() ; ii < bin_vec[it].end() ; ii++ )
      for( std::vector<int>::iterator jj = ii+1 ; jj < bin_vec[it].end() ; jj++ )
      {
        adj_graph[*ii][*jj]++;
        adj_graph[*jj][*ii]++;
      }
  std::vector<bool> dead_flag(pose_num, 0);
  for( int i = 0 ; i < pose_num ; i++ ){
    if( dead_flag[i] == true )
      continue;
    for( int j = i+1 ; j < pose_num ; j++ )
    {
      if( dead_flag[j] == true )
        continue;
      int min_tmp = std::min(votes[i], votes[j]);
      if( (adj_graph[i][j]+0.0) / min_tmp >= 0.75 )
      {
        if( votes[i] > votes[j] )
          dead_flag[j] = true;
        else
        {
          dead_flag[i] = true;
          break;
        }
      }
    }
  }
  std::vector<poseT> refined_poses;
  for( int i = 0 ; i < pose_num ; i++ )
    if( dead_flag[i] == false )
      refined_poses.push_back(all_poses[i]);

  return refined_poses;
}


/*
#pragma omp parallel for
for( int it = 0 ; it < down_num ; it++ )
{
for(int i = 0 ; i < pose_num ; i++ )
{
std::vector<int> idx (1);
std::vector<float> sqrDist (1);
int nres = tree_set[i]->nearestKSearch(down_scene->at(it), 1, idx, sqrDist);
if ( nres >= 1 && sqrDist[0] <= sqrT )
{
bin_vec[it].push_back(i);
//votes[i]++;
}
}

#pragma omp critical
{
for( std::vector<int>::iterator ii = bin_idx.begin() ; ii < bin_idx.end() ; ii++ )
{
votes[*ii]++;
for( std::vector<int>::iterator jj = ii+1 ; jj < bin_idx.end() ; jj++ )
adj_graph[*ii][*jj]++;
}
}
}
*/
