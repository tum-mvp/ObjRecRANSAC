#include "utility.h"

bool exists_test (const std::string& name) {
  if (FILE *file = fopen(name.c_str(), "r")) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

bool exists_dir (const std::string& name)
{
  boost::filesystem::path p( name );
  return boost::filesystem::is_directory(p);
}

double get_wall_time(){
  struct timeval time;
  if (gettimeofday(&time,NULL)){
    //  Handle error
    return 0;
  }
  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

ModelT LoadMesh(std::string filename, std::string label)
{
  ModelT cur_model;
  cur_model.model_mesh = pcl::PolygonMesh::Ptr (new pcl::PolygonMesh());
  pcl::io::loadPolygonFile(filename, *cur_model.model_mesh);

  pcl::PointCloud<myPointXYZ>::Ptr cloud(new pcl::PointCloud<myPointXYZ>());
  pcl::fromPCLPointCloud2(cur_model.model_mesh->cloud, *cloud);

  cur_model.model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>());
  pcl::VoxelGrid<myPointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.004, 0.004, 0.004);
  sor.filter(*cur_model.model_cloud);

  cur_model.model_center = ComputeCentroid(cur_model.model_cloud);
  cur_model.model_label = label;

  return cur_model;
}

pcl::PointCloud<myPointXYZ>::Ptr ComputeCentroid(pcl::PointCloud<myPointXYZ>::Ptr cloud)
{
  double cx=0, cy=0, cz=0;
  int num = cloud->size();
  for(int i=0; i < num ; i++ )
  {
    cx += cloud->at(i).x;
    cy += cloud->at(i).y;
    cz += cloud->at(i).z;
  }
  myPointXYZ centroid;
  centroid.x = cx / num;
  centroid.y = cy / num;
  centroid.z = cz / num;

  pcl::PointCloud<myPointXYZ>::Ptr center_cloud(new pcl::PointCloud<myPointXYZ>());
  center_cloud->clear();
  center_cloud->push_back(centroid);

  return center_cloud;
}

int readCSV(std::string filename, std::string label, std::vector<poseT> &poses)
{
  std::ifstream fp;
  fp.open(filename.c_str());
  if(fp.is_open() == false)
    return 0;

  int num;
  fp >> num;
  for(int i = 0 ; i < num ; i++)
  {
    poseT tmp;
    fp >> tmp.shift(0) >> tmp.shift(1) >> tmp.shift(2);
    float x,y,z,w;
    fp >> x >> y >> z >> w;
    tmp.rotation = Eigen::Quaternionf (w,x,y,z);
    tmp.model_name = label;

    poses.push_back(tmp);
  }
  fp.close();
  return 1;
}

int writeCSV(std::string filename, std::string label, const std::vector<poseT> &poses)
{
  std::ofstream fp;
  fp.open(filename.c_str());
  if( fp.is_open() == false )
    if( fp.is_open() == false )
    {
      std::cerr << "Failed to open files" << std::endl;
      return 0;
    }

  int count=0;
  for( std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++ )
    if( it->model_name == label )
      count++;

  fp << count << std::endl;
  for( std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++ )
  {
    if( it->model_name == label )
    {
      fp << it->shift(0) << " " << it->shift(1) << " " << it->shift(2) << " "
        << it->rotation.x() << " " << it->rotation.y() << " " << it->rotation.z() << " " << it->rotation.w() << std::endl;
      /*
         fp << it->tran(0, 0) << " " << it->tran(0, 1) << " " << it->tran(0, 2) << " " << it->tran(0, 3) << " "
         << it->tran(1, 0) << " " << it->tran(1, 1) << " " << it->tran(1, 2) << " " << it->tran(1, 3) << " "
         << it->tran(2, 0) << " " << it->tran(2, 1) << " " << it->tran(2, 2) << " " << it->tran(2, 3) << " "
         << it->tran(3, 0) << " " << it->tran(3, 1) << " " << it->tran(3, 2) << " " << it->tran(3, 3) << std::endl;
         */
    }
  }
  fp.close();
  return 1;
}

/*
   int readCSV(std::string filename, std::vector< std::vector<float> > &poses)
   {
   std::ifstream fp;
   fp.open(filename.c_str());
   if(fp.is_open() == false)
   return 0;

   int num;
   fp >> num;
   poses.resize(num);
   for(int i = 0 ; i < num ; i++)
   {
   poses[i].resize(7);
   for( int j = 0 ; j < 7 ; j++ )
   fp >> poses[i][j];
   }
   fp.close();
   return 1;
   }

   int writeCSV(std::string filename, const std::vector< std::vector<float> > &poses)
   {
   std::ofstream fp;
   fp.open(filename.c_str());
   if( fp.is_open() == false )
   {
   std::cerr << "Failed to open files" << std::endl;
   return 0;
   }

   int num = poses.size();
   fp << num << std::endl;
   for(int i = 0 ; i < num ; i++)
   {
   for( int j = 0 ; j < poses[i].size() ; j++ )
   fp << poses[i][j] << " ";
   fp << std::endl;
   }
   fp.close();
   return 1;
   }
   */
pcl::PointCloud<myPointXYZ>::Ptr FilterCloud(const pcl::PointCloud<myPointXYZ>::Ptr scene, const pcl::PointCloud<myPointXYZ>::Ptr tran_model, float T)
{
  pcl::search::KdTree<myPointXYZ> tree;
  tree.setInputCloud (tran_model);

  float sqrT = T*T;
  pcl::PointCloud<myPointXYZ>::Ptr filtered_scene(new pcl::PointCloud<myPointXYZ>());
  int num = scene->size();

#pragma omp parallel for
  for(int i = 0 ; i < num ; i++ )
  {
    std::vector<int> indices (1);
    std::vector<float> sqr_dist (1);
    int nres = tree.nearestKSearch(scene->at(i), 1, indices, sqr_dist);
    if ( sqr_dist[0] > sqrT )
    {
#pragma omp critical
      {
        filtered_scene->push_back(scene->at(i));
      }
    }
  }
  return filtered_scene;
}

void splitCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<myPointXYZ>::Ptr link_cloud, pcl::PointCloud<myPointXYZ>::Ptr node_cloud)
{
  for(pcl::PointCloud<PointT>::iterator it = cloud->begin() ; it < cloud->end() ; it++ )
  {
    myPointXYZ tmp;
    tmp.x = it->x;
    tmp.y = it->y;
    tmp.z = it->z;

    //float score = *reinterpret_cast<float*>(&(it->rgba));

    if( it->rgba > 255 )  //link
      link_cloud->push_back(tmp);
    else if( it->rgba > 0 )   //node
      node_cloud->push_back(tmp);
  }
}

float sqrDistPt(const myPointXYZ &pt1, const myPointXYZ &pt2)
{
  float diffx = pt1.x - pt2.x;
  float diffy = pt1.y - pt2.y;
  float diffz = pt1.z - pt2.z;
  return diffx*diffx + diffy*diffy + diffz*diffz;
}

float sqrDistPtT(const PointT &pt1, const PointT &pt2)
{
  float diffx = pt1.x - pt2.x;
  float diffy = pt1.y - pt2.y;
  float diffz = pt1.z - pt2.z;
  return diffx*diffx + diffy*diffy + diffz*diffz;
}

//=========================================================================================================================
/*
   vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts)
   {
   vtkNew<vtkIdTypeArray> cells;
   cells->SetNumberOfValues(numberOfVerts*2);
   vtkIdType* ids = cells->GetPointer(0);
   for (vtkIdType i = 0; i < numberOfVerts; ++i)
   {
   ids[i*2] = 1;
   ids[i*2+1] = i;
   }

   vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
   cellArray->SetCells(numberOfVerts, cells.GetPointer());
   return cellArray;
   }
   */
//*
vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  vtkIdType nr_points = cloud->points.size();

  //vtkNew<vtkPoints> points;
  //points->SetDataTypeToFloat();
  //points->SetNumberOfPoints(nr_points);

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i) {
      //float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      //points->SetPoint(i, point);
      points->InsertNextPoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    }
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud->points[i].x) ||
          !pcl_isfinite (cloud->points[i].y) ||
          !pcl_isfinite (cloud->points[i].z))
        continue;

      //float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      //points->SetPoint(j, point);
      points->InsertNextPoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
      j++;
    }
    //nr_points = j;
    //points->SetNumberOfPoints(nr_points);
  }

  //vtkPolyData *pd = vtkPolyData::New();
  //pd->SetPoints(points);

  //vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  //polyData.TakeReference(pd);
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points);
  //polyData->SetPoints(points.GetPointer());
  //polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}
//*/
//vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(const pcl::PointCloud<PointT>::Ptr cloud)
/*
   vtkPoints* PolyDataFromPointCloud(const pcl::PointCloud<PointT>::Ptr cloud)
   {
   vtkIdType nr_points = cloud->points.size();

   vtkNew<vtkPoints> points;
   points->SetDataTypeToFloat();
   points->SetNumberOfPoints(nr_points);

   if (cloud->is_dense)
   {
   for (vtkIdType i = 0; i < nr_points; ++i) {
   float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
   points->SetPoint(i, point);
   }
   }
   else
   {
   vtkIdType j = 0;    // true point index
   for (vtkIdType i = 0; i < nr_points; ++i)
   {
// Check if the point is invalid
if (!pcl_isfinite (cloud->points[i].x) ||
!pcl_isfinite (cloud->points[i].y) ||
!pcl_isfinite (cloud->points[i].z))
continue;

float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
points->SetPoint(j, point);
j++;
}
nr_points = j;
points->SetNumberOfPoints(nr_points);
}

//vtkPolyData *pd = vtkPolyData::New();
//pd->SetPoints(points.GetPointer());
//vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
//polyData.TakeReference(pd);
//polyData->SetPoints(points.GetPointer());
//polyData->SetVerts(NewVertexCells(nr_points));
//return polyData;
return points.GetPointer();
}
*/
/*
   void PolyDataFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
   {
   vtkIdType nr_points = cloud->points.size();

   vtkNew<vtkPoints> points;
   points->SetDataTypeToFloat();
   points->SetNumberOfPoints(nr_points);

   if (cloud->is_dense)
   {
   for (vtkIdType i = 0; i < nr_points; ++i) {
   float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
   points->SetPoint(i, point);
   }
   }
   else
   {
   vtkIdType j = 0;    // true point index
   for (vtkIdType i = 0; i < nr_points; ++i)
   {
// Check if the point is invalid
if (!pcl_isfinite (cloud->points[i].x) ||
!pcl_isfinite (cloud->points[i].y) ||
!pcl_isfinite (cloud->points[i].z))
continue;

float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
points->SetPoint(j, point);
j++;
}
nr_points = j;
points->SetNumberOfPoints(nr_points);
}

//vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
//polyData->SetPoints(points.GetPointer());
//polyData->SetVerts(NewVertexCells(nr_points));
//return polyData;

//return points.GetPointer();
}
*/


