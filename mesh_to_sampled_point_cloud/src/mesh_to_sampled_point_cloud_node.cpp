

#include "mesh_to_sampled_point_cloud/mesh_to_sampled_point_cloud.h"
#include "mesh_to_sampled_point_cloud/ConvertMeshToSampledPointCloud.h"

#include <ros/ros.h>


bool convertMeshToSampledPointCloudCallback(
  mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Request& request,
  mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Response& response)
{

  std::unique_ptr<mesh_to_sampled_point_cloud::MeshToSampledPointCloud> mesh_to_sampled_point_cloud_converter;

  try
  {
    // import and convert sampled point cloud
    mesh_to_sampled_point_cloud_converter = std::make_unique<mesh_to_sampled_point_cloud::MeshToSampledPointCloud>(
      request.input_file, request.num_sample_points, request.voxel_filter_leaf_size, request.translation,
      request.rotation_euler_deg, request.scale_factor, request.write_normals, request.write_colors);
  } catch (std::invalid_argument& e)
  {
    response.result = mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Response::LOAD_FILE_ERROR;
    response.error_msg = e.what();
    return true;
  } catch (std::runtime_error& e)
  {
    response.result = mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Response::CONVERSION_ERROR;
    response.error_msg = e.what();
    return true;
  }

  if (!mesh_to_sampled_point_cloud_converter)
  {
    return false;
  }

  try
  {
    // save point cloud
    mesh_to_sampled_point_cloud_converter->savePointCloudToPCDFile(request.output_file);
  } catch (std::invalid_argument& e)
  {
    response.result = mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Response::SAVE_FILE_ERROR;
    response.error_msg = e.what();
    return true;
  }
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::ServiceServer convert_mesh_to_sampled_point_cloud_service_server =
    pnh.advertiseService("convert_mesh_to_sampled_point_cloud", convertMeshToSampledPointCloudCallback);

  ros::spin();


  return 0;
}

