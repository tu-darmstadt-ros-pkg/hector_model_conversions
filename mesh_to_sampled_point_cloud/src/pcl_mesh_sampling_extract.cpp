
#include "mesh_to_sampled_point_cloud/pcl_mesh_sampling_extract.h"

namespace mesh_to_sampled_point_cloud
{

// ===== Constructor and public methods extracted from pcl_mesh_sampling main method ===== //


PCLMeshSamplingExtract::PCLMeshSamplingExtract(int num_sample_points, float voxel_filter_leaf_size,
                                               vtkSmartPointer<vtkTransform> transform, bool write_normals,
                                               bool write_colors)
  : num_sample_points_(num_sample_points), voxel_filter_leaf_size_(voxel_filter_leaf_size), transform_(transform),
    write_normals_(write_normals), write_colors_(write_colors)
{

  // init data structures
  polydata_ = vtkSmartPointer<vtkPolyData>::New();
  point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  filtered_point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
}


std::string PCLMeshSamplingExtract::importFile(const std::string& input_file_name)
{
  // get file extension
  auto pos = input_file_name.find('.');
  if (pos == std::string::npos)
  {
    return "Error while importing file \"" + input_file_name + "\": no file extension found!";
  }
  std::string file_extension = input_file_name.substr(pos);

  // read file depending on file extension
  vtkSmartPointer<vtkAbstractPolyDataReader> reader;
  if (file_extension == ".ply")
  {
    reader = vtkSmartPointer<vtkPLYReader>::New();
  }
  else if (file_extension == ".obj")
  {
    reader = vtkSmartPointer<vtkOBJReader>::New();
  }
  else
  {
    return "Unknown file extension \"" + file_extension + "\" of input_file_name \"" + input_file_name + "\".";
  }

  reader->SetFileName(input_file_name.c_str());
  reader->Update();
  polydata_ = reader->GetOutput();

  return "";
}


bool PCLMeshSamplingExtract::executeSampling()
{
  // scale and transform polydata
  scaleAndTransform(transform_);

  // ensure that polygons in polydata are triangles
  filterTriangles();

  // sampling and conversion into pointcloud
  uniform_sampling(polydata_, num_sample_points_, write_normals_, write_colors_, *point_cloud_);

  // filter pointcloud using a voxelgrid
  filterPointCloud();

  return true;
}


std::string PCLMeshSamplingExtract::savePCDFile(const std::string& output_file_name)
{
  auto pos = output_file_name.find('.');
  if (pos != std::string::npos && output_file_name.substr(pos) != ".pcd")
  {
    return "Error while saving pcd file \"" + output_file_name + "\": output_file_name has wrong file extension!";
  }

  int save_result;

  if (write_normals_ && write_colors_)
  {
    save_result = pcl::io::savePCDFileASCII(output_file_name, *filtered_point_cloud_);
  }
  else if (write_normals_)
  {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyzn(new pcl::PointCloud<pcl::PointNormal>);
    // Strip uninitialized colors from cloud:
    pcl::copyPointCloud(*filtered_point_cloud_, *cloud_xyzn);
    save_result = pcl::io::savePCDFileASCII(output_file_name, *cloud_xyzn);
  }
  else if (write_colors_)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Strip uninitialized normals from cloud:
    pcl::copyPointCloud(*filtered_point_cloud_, *cloud_xyzrgb);
    save_result = pcl::io::savePCDFileASCII(output_file_name, *cloud_xyzrgb);
  }
  else // !write_normals_ && !write_colors_
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    // Strip uninitialized normals and colors from cloud:
    pcl::copyPointCloud(*filtered_point_cloud_, *cloud_xyz);
    save_result = pcl::io::savePCDFileASCII(output_file_name, *cloud_xyz);
  }

  if (save_result != 0)
  {
    return "Error while saving PCD file. Please see PCL error for more information.";
  }

  return "";
}


std::shared_ptr<pcl::PCLPointCloud2> PCLMeshSamplingExtract::getSampledPointCloud()
{
  std::shared_ptr<pcl::PCLPointCloud2> point_cloud_2 = std::make_shared<pcl::PCLPointCloud2>();

  if (write_normals_ && write_colors_)
  {
    pcl::toPCLPointCloud2(*filtered_point_cloud_, *point_cloud_2);
  }
  else if (write_normals_)
  {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyzn(new pcl::PointCloud<pcl::PointNormal>);
    // Strip uninitialized colors from cloud:
    pcl::copyPointCloud(*filtered_point_cloud_, *cloud_xyzn);
    pcl::toPCLPointCloud2(*cloud_xyzn, *point_cloud_2);
  }
  else if (write_colors_)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Strip uninitialized normals from cloud:
    pcl::copyPointCloud(*filtered_point_cloud_, *cloud_xyzrgb);
    pcl::toPCLPointCloud2(*cloud_xyzrgb, *point_cloud_2);
  }
  else // !write_normals_ && !write_colors_
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    // Strip uninitialized normals and colors from cloud:
    pcl::copyPointCloud(*filtered_point_cloud_, *cloud_xyz);
    pcl::toPCLPointCloud2(*cloud_xyz, *point_cloud_2);;
  }

  return point_cloud_2;
}


std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PCLMeshSamplingExtract::getSampledPointCloudXYZ()
{
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  // Strip normals and colors from cloud:
  pcl::copyPointCloud(*filtered_point_cloud_, *cloud_xyz);
  return cloud_xyz;
}


std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PCLMeshSamplingExtract::getSampledPointCloudXYZN()
{
  if (!write_normals_)
  {
    std::cout << "Parameter \"write_normals\" must be set for calling this function" << std::endl;
  }

  std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> cloud_xyzn = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  // Strip colors from cloud:
  pcl::copyPointCloud(*filtered_point_cloud_, *cloud_xyzn);
  return cloud_xyzn;
}



// ===== pcl_mesh_sampling methods ===== //


double PCLMeshSamplingExtract::uniform_deviate(int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

void PCLMeshSamplingExtract::randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1,
                                                 float c2, float c3, float r1, float r2, Eigen::Vector3f& p)
{
  float r1sqr = std::sqrt(r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
}

void PCLMeshSamplingExtract::randPSurface(vtkPolyData* polydata, std::vector<double>* cumulativeAreas,
                                          double totalArea, Eigen::Vector3f& p, bool calcNormal, Eigen::Vector3f& n,
                                          bool calcColor, Eigen::Vector3f& c)
{
  float r = static_cast<float> (uniform_deviate(rand()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
  vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType* ptIds = nullptr;
  polydata->GetCellPoints(el, npts, ptIds);
  polydata->GetPoint(ptIds[0], A);
  polydata->GetPoint(ptIds[1], B);
  polydata->GetPoint(ptIds[2], C);
  if (calcNormal)
  {
    // OBJ: Vertices are stored in a counter-clockwise order by default
    Eigen::Vector3f v1 = Eigen::Vector3f(A[0], A[1], A[2]) - Eigen::Vector3f(C[0], C[1], C[2]);
    Eigen::Vector3f v2 = Eigen::Vector3f(B[0], B[1], B[2]) - Eigen::Vector3f(C[0], C[1], C[2]);
    n = v1.cross(v2);
    n.normalize();
  }
  float r1 = static_cast<float> (uniform_deviate(rand()));
  float r2 = static_cast<float> (uniform_deviate(rand()));
  randomPointTriangle(float(A[0]), float(A[1]), float(A[2]),
                      float(B[0]), float(B[1]), float(B[2]),
                      float(C[0]), float(C[1]), float(C[2]), r1, r2, p);

  if (calcColor)
  {
    vtkUnsignedCharArray* const colors = vtkUnsignedCharArray::SafeDownCast(polydata->GetPointData()->GetScalars());
    if (colors && colors->GetNumberOfComponents() == 3)
    {
      double cA[3], cB[3], cC[3];
      colors->GetTuple(ptIds[0], cA);
      colors->GetTuple(ptIds[1], cB);
      colors->GetTuple(ptIds[2], cC);

      randomPointTriangle(float(cA[0]), float(cA[1]), float(cA[2]),
                          float(cB[0]), float(cB[1]), float(cB[2]),
                          float(cC[0]), float(cC[1]), float(cC[2]), r1, r2, c);
    }
    else
    {
      static bool printed_once = false;
      if (!printed_once)
        PCL_WARN("Mesh has no vertex colors, or vertex colors are not RGB!");
      printed_once = true;
    }
  }
}


void PCLMeshSamplingExtract::uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, std::size_t n_samples,
                                              bool calc_normal, bool calc_color,
                                              pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out)
{
  polydata->BuildCells();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
  vtkIdType npts = 0, * ptIds = nullptr;
  std::size_t cellId = 0;
  for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); cellId++)
  {
    polydata->GetPoint(ptIds[0], p1);
    polydata->GetPoint(ptIds[1], p2);
    polydata->GetPoint(ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
    cumulativeAreas[cellId] = totalArea;
  }

  cloud_out.points.resize(n_samples);
  cloud_out.width = static_cast<std::uint32_t> (n_samples);
  cloud_out.height = 1;

  for (std::size_t i = 0; i < n_samples; i++)
  {
    Eigen::Vector3f p;
    Eigen::Vector3f n(0, 0, 0);
    Eigen::Vector3f c(0, 0, 0);
    randPSurface(polydata, &cumulativeAreas, totalArea, p, calc_normal, n, calc_color, c);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
    if (calc_normal)
    {
      cloud_out.points[i].normal_x = n[0];
      cloud_out.points[i].normal_y = n[1];
      cloud_out.points[i].normal_z = n[2];
    }
    if (calc_color)
    {
      cloud_out.points[i].r = static_cast<std::uint8_t>(c[0]);
      cloud_out.points[i].g = static_cast<std::uint8_t>(c[1]);
      cloud_out.points[i].b = static_cast<std::uint8_t>(c[2]);
    }
  }
}


// ===== new transform method ===== //
void PCLMeshSamplingExtract::scaleAndTransform(vtkSmartPointer<vtkTransform> transform)
{
  // set up transform filter
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetInputData(polydata_);
  transformFilter->SetTransform(transform);
  transformFilter->Update();

  // apply transform filter
  vtkSmartPointer<vtkPolyDataMapper> transformMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  transformMapper->SetInputConnection(transformFilter->GetOutputPort());
  transformMapper->Update();
  polydata_ = transformMapper->GetInput();
}


// ===== private methods extracted from pcl_mesh_sampling main method ===== //

void PCLMeshSamplingExtract::filterTriangles()
{
  //make sure that the polygons are triangles!
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInputData(polydata_);
  triangleFilter->Update();

  vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
  triangleMapper->Update();
  polydata_ = triangleMapper->GetInput();
}

void PCLMeshSamplingExtract::filterPointCloud()
{
  // Voxelgrid for filtering point_cloud using a grid
  pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid_;
  grid_.setInputCloud(point_cloud_);
  grid_.setLeafSize(voxel_filter_leaf_size_, voxel_filter_leaf_size_, voxel_filter_leaf_size_);

  // filtered cloud
  grid_.filter(*filtered_point_cloud_);
}
} // end namespace mesh_to_sampled_point_cloud