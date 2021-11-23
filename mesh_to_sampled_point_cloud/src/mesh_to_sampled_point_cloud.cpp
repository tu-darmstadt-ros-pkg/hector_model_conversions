
#include "mesh_to_sampled_point_cloud/mesh_to_sampled_point_cloud.h"

namespace mesh_to_sampled_point_cloud
{

MeshToSampledPointCloud::MeshToSampledPointCloud(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
{
  convert_mesh_to_sampled_point_cloud_service_server_ =
    pnh_.advertiseService("convert_mesh_to_sampled_point_cloud",
                          &MeshToSampledPointCloud::convertMeshToSampledPointCloudCallback, this);
}

bool MeshToSampledPointCloud::convertMeshToSampledPointCloudCallback(
  mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Request& request,
  mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Response& response)
{

  PCLMeshSamplingExtract mesh_sampling_extract(request.num_sample_points, request.voxel_filter_leaf_size,
                                               convertTransform(request.translation, request.rotation_euler_deg,
                                                                request.scale_factor),
                                               request.write_normals, request.write_colors);

  // import file
  std::string import_result = mesh_sampling_extract.importFile(request.input_file);
  if (!import_result.empty())
  {
    response.result = mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Response::LOAD_FILE_ERROR;
    response.error_msg = import_result;
    return true;
  }

  // sampling and conversion
  if (!mesh_sampling_extract.executeSampling())
  {
    response.result = mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Response::CONVERSION_ERROR;
    response.error_msg = "Error while converting and sampling pointcloud.";
    return true;
  }

  // save file
  std::string save_result = mesh_sampling_extract.savePCDFile(request.output_file);
  if (!save_result.empty())
  {
    response.result = mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Response::SAVE_FILE_ERROR;
    response.error_msg = save_result;
    return true;
  }

  // always return true so that caller receives response
  return true;
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