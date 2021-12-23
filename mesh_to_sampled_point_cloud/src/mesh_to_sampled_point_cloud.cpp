
#include "mesh_to_sampled_point_cloud/mesh_to_sampled_point_cloud.h"

namespace mesh_to_sampled_point_cloud
{

MeshToSampledPointCloud::MeshToSampledPointCloud(const std::string& input_file, int num_sample_points,
                                                 float voxel_filter_leaf_size,
                                                 geometry_msgs::Vector3 translation,
                                                 geometry_msgs::Vector3 rotation_euler_deg,
                                                 float scale_factor, bool write_normals, bool write_colors)
  : mesh_sampling_extract_(num_sample_points, voxel_filter_leaf_size,
                           convertTransform(translation, rotation_euler_deg, scale_factor),
                           write_normals, write_colors)
{

  // import file
  std::string import_result = mesh_sampling_extract_.importFile(input_file);
  if (!import_result.empty())
  {
    throw std::invalid_argument("LOAD_FILE_ERROR: " + import_result);
  }

  // sampling and conversion
  if (!mesh_sampling_extract_.executeSampling())
  {
    throw std::runtime_error("Error while converting and sampling pointcloud.");
  }
}

std::shared_ptr<pcl::PCLPointCloud2> MeshToSampledPointCloud::getPointCloud()
{
  return mesh_sampling_extract_.getSampledPointCloud();
}

void MeshToSampledPointCloud::savePointCloudToPCDFile(const std::string& output_file)
{
  // save file
  std::string save_result = mesh_sampling_extract_.savePCDFile(output_file);
  if (!save_result.empty())
  {
    throw std::invalid_argument("SAVE_FILE_ERROR: " + save_result);
  }
}


vtkSmartPointer<vtkTransform>
MeshToSampledPointCloud::convertTransform(geometry_msgs::Vector3 translation, geometry_msgs::Vector3 rotation_euler_deg,
                                          double scale_factor)
{
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

  transform->Translate(translation.x, translation.y, translation.z);

  transform->RotateX(rotation_euler_deg.x);
  transform->RotateY(rotation_euler_deg.y);
  transform->RotateZ(rotation_euler_deg.z);

  // if scale factor is 0, no data will be available, so set it to 1
  if (scale_factor == 0.0)
  {
    scale_factor = 1.0;
    ROS_WARN_STREAM_NAMED("mesh_to_sampled_point_cloud", "Scale factor was 0.0, instead 1.0 is used.");
  }

  transform->Scale(scale_factor, scale_factor, scale_factor);

  return transform;
}
} // end namespace mesh_to_sampled_point_cloud